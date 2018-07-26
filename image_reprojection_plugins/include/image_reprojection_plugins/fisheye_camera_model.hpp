#ifndef IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP

#include <cmath>
#include <vector>

#include <image_reprojection/camera_model.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <param_utilities/param_utilities.hpp>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class FisheyeCameraModel : public image_reprojection::CameraModel {
public:
  FisheyeCameraModel() {}

  virtual ~FisheyeCameraModel() {}

private:
  virtual void onInit() { ROS_INFO_STREAM(getName() << " has been initialized"); }

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
    // reproject 2D points in the image coordinate to the camera coordinate
    /*
    cv::fisheye::undistortPointsTo3D(src.reshape(2, src.total()), cam, camera_matrix_,
                                     dist_coeffs_);
                                     */
    dst = dst.reshape(3, src.size().height);

    // update mask to indicate valid output points.
    // an output point is valid when
    //   - corresponding input pixel is valid (input mask is valid)
    //   - corresponding input pixel is in image frame
    //   - output ray direction is in field of view
    for (int x = 0; x < mask.size().width; ++x) {
      for (int y = 0; y < mask.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Point2f &s(src.at< cv::Point2f >(y, x));
        const cv::Point3f &d(dst.at< cv::Point3f >(y, x));
        m = (m != 0 && frame_.contains(s) && std::acos(d.z / cv::norm(d)) < fov_) ? 1 : 0;
      }
    }
  }

  virtual void fromCameraInfo(const sensor_msgs::CameraInfo &camera_info) {
    // TODO: implement!!
    // Note: load **FOV** as well as cam mat and dist coeffs
  }

  virtual sensor_msgs::CameraInfoPtr toCameraInfo() const {
    // TODO: implement!!
  }

private:
  // parameters
  cv::Matx33d camera_matrix_;
  std::vector< double > dist_coeffs_;
  cv::Rect_< float > frame_;
  double fov_;
};

} // namespace image_reprojection_plugins

#endif /* IMAGE_REPROJECTION_PLUGINS_FISHEYE_CAMERA_MODEL_HPP */
