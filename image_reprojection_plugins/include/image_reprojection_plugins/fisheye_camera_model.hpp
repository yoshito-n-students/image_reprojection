#ifndef IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP

#include <cmath>
#include <limits>
#include <vector>

#include <image_reprojection/camera_model.hpp>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/calib3d/calib3d.hpp> //for cv::Affine3d
#include <opencv2/core/core.hpp>

#include <boost/make_shared.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace image_reprojection_plugins {

class FisheyeCameraModel : public image_reprojection::CameraModel {
public:
  FisheyeCameraModel() {}

  virtual ~FisheyeCameraModel() {}

  virtual void fromCameraInfo(const sensor_msgs::CameraInfo &camera_info) override {
    // assert distortion type and number of distortion parameters
    CV_Assert(camera_info.distortion_model == "fisheye");
    CV_Assert(camera_info.D.empty() || camera_info.D.size() == 4);

    boost::unique_lock<boost::shared_mutex> write_lock(mutex_);

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

    // copy ROI info in full resolution
    frame_.x = camera_info.roi.x_offset;
    frame_.y = camera_info.roi.y_offset;
    frame_.width = (camera_info.roi.width == 0 ? camera_info.width : camera_info.roi.width);
    frame_.height = (camera_info.roi.height == 0 ? camera_info.height : camera_info.roi.height);

    // adust image scaling
    if (camera_info.binning_x != 0 && camera_info.binning_x != 1) {
      camera_matrix_(0, 0) /= camera_info.binning_x;
      camera_matrix_(0, 1) /= camera_info.binning_x;
      camera_matrix_(0, 2) /= camera_info.binning_x;
      frame_.x /= camera_info.binning_x;
      frame_.width /= camera_info.binning_x;
    }
    if (camera_info.binning_y != 0 && camera_info.binning_y != 1) {
      camera_matrix_(1, 0) /= camera_info.binning_y;
      camera_matrix_(1, 1) /= camera_info.binning_y;
      camera_matrix_(1, 2) /= camera_info.binning_y;
      frame_.y /= camera_info.binning_y;
      frame_.height /= camera_info.binning_y;
    }

    // copy distortion coefficients which are independent from image resolution
    if (camera_info.D.empty()) {
      dist_coeffs_.resize(4, 0.);
    } else {
      dist_coeffs_ = camera_info.D;
    }
  }

  virtual sensor_msgs::CameraInfoPtr toCameraInfo() const {
    boost::shared_lock<boost::shared_mutex> read_lock(mutex_);
    return boost::make_shared<sensor_msgs::CameraInfo>(camera_info_);
  }

private:
  virtual void onInit() override {
    boost::unique_lock<boost::shared_mutex> write_lock(mutex_);

    ros::NodeHandle &pnh = getPrivateNodeHandle();
    fov_ = pnh.param("fov", M_PI);
    skew_ = pnh.param("skew", 0.);
  }

  virtual void onProject3dToPixel(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const override {
    boost::shared_lock<boost::shared_mutex> read_lock(mutex_);

    // allocate dst pixels
    dst.create(src.size(), CV_32FC2);

    // update dst points when
    //   - corresponding input point is valid (input mask is valid)
    // if update of dst pixel succeeded, set corresponding mask to 1, otherwise 0
    mask.forEach<uchar>([this, &src, &dst](uchar &m, const int *const pos) {
      if (m != 0) {
        const cv::Point3f &s = *src.ptr<cv::Point3f>(pos[0], pos[1]);
        cv::Point2f &d = *dst.ptr<cv::Point2f>(pos[0], pos[1]);
        m = project3dPointToPixel(s, d) ? 1 : 0;
      }
    });
  }

  bool project3dPointToPixel(const cv::Point3f &src, cv::Point2f &dst) const {
    // angle between source point and z-axis
    const double theta = std::acos(src.z / cv::norm(src));

    // check source point is in field of view
    if (theta > fov_ / 2.) {
      return false;
    }

    // distance between distorted point and z-axis
    double theta_d;
    {
      const double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2,
                   theta8 = theta6 * theta2;
      theta_d = theta * (1. + dist_coeffs_[0] * theta2 + dist_coeffs_[1] * theta4 +
                         dist_coeffs_[2] * theta6 + dist_coeffs_[3] * theta8);
    }

    // distort point
    cv::Point2d distorted;
    {
      const double scale_d = theta_d / cv::sqrt(src.x * src.x + src.y * src.y);
      distorted.x = scale_d * src.x;
      distorted.y = scale_d * src.y;
      // distorted.z = 1.;
    }

    // transform distorted point to pixel coordinate
    dst.x = camera_matrix_(0, 0) * (distorted.x + skew_ * distorted.y) + camera_matrix_(0, 2);
    dst.y = camera_matrix_(1, 1) * distorted.y + camera_matrix_(1, 2);

    // check dst pixel is in image frame
    if (!frame_.contains(dst)) {
      return false;
    }

    return true;
  }

  virtual void onProjectPixelTo3dRay(const cv::Mat &src, cv::Mat &dst,
                                     cv::Mat &mask) const override {
    boost::shared_lock<boost::shared_mutex> read_lock(mutex_);

    // allocate dst points
    dst.create(src.size(), CV_32FC3);

    // update dst points when
    //   - corresponding input pixel is valid (input mask is valid)
    // if update of dst point succeeded, set corresponding mask to 1, otherwise 0
    mask.forEach<uchar>([this, &src, &dst](uchar &m, const int *const pos) {
      if (m != 0) {
        const cv::Point2f &s = *src.ptr<cv::Point2f>(pos[0], pos[1]);
        cv::Point3f &d = *dst.ptr<cv::Point3f>(pos[0], pos[1]);
        m = projectPixelPointTo3dRay(s, d) ? 1 : 0;
      }
    });
  }

  bool projectPixelPointTo3dRay(const cv::Point2f &src, cv::Point3f &dst) const {
    // check source pixel is in image frame
    if (!frame_.contains(src)) {
      return false;
    }

    // distorted point in camera coordinate
    const cv::Point3d distorted((src.x - camera_matrix_(0, 2)) / camera_matrix_(0, 0) -
                                    skew_ * (src.y - camera_matrix_(1, 2)) / camera_matrix_(1, 1),
                                (src.y - camera_matrix_(1, 2)) / camera_matrix_(1, 1), 1.);
    const double theta_d = cv::sqrt(distorted.x * distorted.x + distorted.y * distorted.y);

    // angle of undistorted point from z-axis
    double theta = theta_d;
    if (theta_d > 1e-8) {
      // newton's method:
      // to find theta giving f(theta) = 0, where
      //   f(theta) = theta * (1 + k0*theta^2 + ... + k3*theta^8) - theta_d
      // update theta iteratively by
      //   theta <- theta - theta_fix
      //   theta_fix = f(theta) / f'(theta)
      for (int i = 0; i < 10; ++i) {
        const double theta2 = theta * theta, theta4 = theta2 * theta2, theta6 = theta4 * theta2,
                     theta8 = theta6 * theta2;
        const double k0_theta2 = dist_coeffs_[0] * theta2, k1_theta4 = dist_coeffs_[1] * theta4,
                     k2_theta6 = dist_coeffs_[2] * theta6, k3_theta8 = dist_coeffs_[3] * theta8;
        const double theta_fix =
            (theta * (1 + k0_theta2 + k1_theta4 + k2_theta6 + k3_theta8) - theta_d) /
            (1 + 3 * k0_theta2 + 5 * k1_theta4 + 7 * k2_theta6 + 9 * k3_theta8);
        theta -= theta_fix;
        if (std::abs(theta_fix) < std::numeric_limits<double>::epsilon()) {
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
  mutable boost::shared_mutex mutex_;

  // ros standard camera info
  sensor_msgs::CameraInfo camera_info_;
  cv::Matx33d camera_matrix_;
  std::vector<double> dist_coeffs_;
  cv::Rect_<float> frame_;

  // additional info from rosparam
  double fov_;
  double skew_;
};

} // namespace image_reprojection_plugins

#endif /* IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP */
