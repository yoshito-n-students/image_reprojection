#ifndef IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP

#include <cmath>
#include <limits>
#include <vector>

#include <image_reprojection/camera_model.hpp>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <boost/make_shared.hpp>

namespace image_reprojection_plugins {

class FisheyeCameraModel : public image_reprojection::CameraModel {
public:
  FisheyeCameraModel() {}

  virtual ~FisheyeCameraModel() {}

  virtual void fromCameraInfo(const sensor_msgs::CameraInfo &camera_info) {
    // assert distortion type and number of distortion parameters
    CV_Assert(camera_info.distortion_model == "fisheye");
    CV_Assert(camera_info.D.size() == 0 /* no distortion & fov */ ||
              camera_info.D.size() == 1 /* fov only */ ||
              camera_info.D.size() == 4 /* distortion only */ ||
              camera_info.D.size() == 5 /* distortion & fov */);

    // copy entire camera info to return it via toCameraInfo()
    camera_info_ = camera_info;

    // copy full resolution camera matrix
    std::copy(camera_info.K.begin(), camera_info.K.end(), camera_matrix_.val);

    // adust offset of principal points.
    // (this should be performed against full camera matrix
    // because offsets are written in full resolution)
    if (camera_info.roi.x_offset != 0) {
      camera_matrix_(0, 2) -= camera_info.roi.x_offset;
    }
    if (camera_info.roi.y_offset != 0) {
      camera_matrix_(1, 2) -= camera_info.roi.y_offset;
    }

    // adust image scaling
    if (camera_info.binning_x != 0 && camera_info.binning_x != 1) {
      camera_matrix_(0, 0) /= camera_info.binning_x;
      camera_matrix_(0, 1) /= camera_info.binning_x;
      camera_matrix_(0, 2) /= camera_info.binning_x;
    }
    if (camera_info.binning_y != 0 && camera_info.binning_y != 1) {
      camera_matrix_(1, 0) /= camera_info.binning_y;
      camera_matrix_(1, 1) /= camera_info.binning_y;
      camera_matrix_(1, 2) /= camera_info.binning_y;
    }

    // copy distortion coefficients which are independent from image resolution
    // (do not copy the last element of camera_info.D because it is field of view)
    switch (camera_info.D.size()) {
    case 0: /* no distortion & fov */
      dist_coeffs_.resize(4, 0.);
      fov_ = M_PI;
      break;
    case 1: /* fov only */
      dist_coeffs_.resize(4, 0.);
      fov_ = camera_info.D[0];
      break;
    case 4: /* distortion only */
      dist_coeffs_ = camera_info.D;
      fov_ = M_PI;
      break;
    case 5: /* distortion & fov */
      dist_coeffs_.assign(camera_info.D.begin(), camera_info.D.begin() + 4);
      fov_ = camera_info.D[4];
      break;
    default: {
      static const bool SHOULD_NOT_REACH_HERE_BUG(false);
      CV_Assert(SHOULD_NOT_REACH_HERE_BUG);
      break;
    }
    }

    // copy image size info
    frame_.x = camera_info.roi.x_offset;
    frame_.y = camera_info.roi.y_offset;
    frame_.width = (camera_info.roi.width == 0 ? camera_info.width : camera_info.roi.width);
    frame_.height = (camera_info.roi.height == 0 ? camera_info.height : camera_info.roi.height);
  }

  virtual sensor_msgs::CameraInfoPtr toCameraInfo() const {
    return boost::make_shared< sensor_msgs::CameraInfo >(camera_info_);
  }

private:
  virtual void onProject3dToPixel(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    // project 3D points in the camera coordinate into the 2D image coordinate
    cv::fisheye::projectPoints(src.reshape(3, src.total()), dst, cv::Vec3d::all(0.),
                               cv::Vec3d::all(0.), camera_matrix_, dist_coeffs_);
    dst = dst.reshape(2, src.size().height);

    // update mask to indicate valid output points.
    // an output point is valid when
    //   - corresponding input point is valid (input mask is valid)
    //   - corresponding input point is in field of view
    //   - output pixel is in the image frame
    for (int x = 0; x < mask.size().width; ++x) {
      for (int y = 0; y < mask.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Point3f &s(src.at< cv::Point3f >(y, x));
        const cv::Point2f &d(dst.at< cv::Point2f >(y, x));
        m = (m != 0 && std::acos(s.z / cv::norm(s)) < fov_ && frame_.contains(d)) ? 1 : 0;
      }
    }
  }

  virtual void onProjectPixelTo3dRay(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    // allocate dst points
    dst.create(src.size(), CV_32FC3);

    // update dst points when
    //   - corresponding input pixel is valid (input mask is valid)
    //   - corresponding input pixel is in image frame
    // if update of dst point succeeded, set corresponding mask to 1, otherwise 0
    for (int x = 0; x < src.size().width; ++x) {
      for (int y = 0; y < src.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Point2f &s(src.at< cv::Point2f >(y, x));
        cv::Point3f &d(dst.at< cv::Point3f >(y, x));
        m = (m != 0 && frame_.contains(s) && projectPixelPointTo3dRay(s, d)) ? 1 : 0;
      }
    }
  }

  bool projectPixelPointTo3dRay(const cv::Point2f &src, cv::Point3f &dst) const {
    // distorted point in camera coordinate
    const cv::Point3d distorted((src.x - camera_matrix_(0, 2)) / camera_matrix_(0, 0),
                                (src.y - camera_matrix_(1, 2)) / camera_matrix_(1, 1), 1.);

    // angles of distorted/undistorted points from z-axis
    const double theta_d(cv::sqrt(distorted.x * distorted.x + distorted.y * distorted.y));
    double theta(theta_d);
    if (theta_d > 1e-8) {
      // newton's method:
      // to find theta giving f(theta) = 0, where
      //   f(theta) = theta * (1 + k0*theta^2 + ... + k3*theta^8) - theta_d
      // update theta iteratively by
      //   theta <- theta - theta_fix
      //   theta_fix = f(theta) / f'(theta)
      for (int i = 0; i < 10; ++i) {
        const double theta2(theta * theta), theta4(theta2 * theta2), theta6(theta4 * theta2),
            theta8(theta6 * theta2);
        const double k0_theta2(dist_coeffs_[0] * theta2), k1_theta4(dist_coeffs_[1] * theta4),
            k2_theta6(dist_coeffs_[2] * theta6), k3_theta8(dist_coeffs_[3] * theta8);
        const double theta_fix(
            (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
            (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8));
        theta -= theta_fix;
        if (std::fabs(theta_fix) < std::numeric_limits< double >::epsilon()) {
          break;
        }
      }
    }

    // check undistorted point is in field of view
    if (theta > fov_ / 2.) {
      return false;
    }

    // undistort point
    const cv::Vec3d axis_rot(-distorted.y, distorted.x, 0.);
    const cv::Vec3d vec_rot(theta * cv::normalize(axis_rot));
    dst = cv::Affine3d(vec_rot).rotation() * cv::Point3d(0., 0., 1.);

    return true;
  }

private:
  // parameters
  sensor_msgs::CameraInfo camera_info_;
  cv::Matx33d camera_matrix_;
  std::vector< double > dist_coeffs_;
  double fov_;
  cv::Rect_< float > frame_;
};

} // namespace image_reprojection_plugins

#endif /* IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP */
