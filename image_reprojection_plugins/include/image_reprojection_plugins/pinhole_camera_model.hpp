#ifndef IMAGE_REPROJECTION_PLUGINS_PINHOLE_CAMERA_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_PINHOLE_CAMERA_MODEL_HPP

#include <cmath>
#include <vector>

#include <image_reprojection/camera_model.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection_plugins {

class PinholeCameraModel : public image_reprojection::CameraModel {
public:
  PinholeCameraModel() {}

  virtual ~PinholeCameraModel() {}

private:
  virtual void onInit() { ROS_INFO_STREAM(getName() << " has been initialized"); }

  virtual void onProject3dToPixel(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    // project 3D points in the camera coordinate into the 2D image coordinate
    cv::projectPoints(src.reshape(3, src.total()), cv::Vec3d::all(0.), cv::Vec3d::all(0.),
                      camera_matrix_, dist_coeffs_, dst);
    dst = dst.reshape(2, src.size().height);

    // update mask to indicate valid output points.
    // an output point is valid when
    //   - corresponding input point is valid (input mask is valid)
    //   - corresponding input point is in front of the camera
    //   - output pixel is in the image frame
    for (int x = 0; x < mask.size().width; ++x) {
      for (int y = 0; y < mask.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Point3f &s(src.at< cv::Point3f >(y, x));
        const cv::Point2f &d(dst.at< cv::Point2f >(y, x));
        m = (m != 0 && s.z >= 0 && frame_.contains(d)) ? 1 : 0;
      }
    }
  }

  virtual void onProjectPixelTo3dRay(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    // reproject 2D points in the image coordinate to the camera coordinate
    cv::Mat dst_2d;
    cv::undistortPoints(src.reshape(2, src.total()), dst_2d, camera_matrix_, dist_coeffs_);
    dst_2d = dst_2d.reshape(2, src.size().height);

    // add z-channel to the points in the camera coordinate
    dst.create(dst_2d.size(), CV_32FC3);
    // copy xy-channels
    dst_2d.reshape(1, dst_2d.total()).copyTo(dst.reshape(1, dst.total()).colRange(0, 2));
    // fill z-channel to 1.0
    dst.reshape(1, dst.total()).col(2).setTo(1.);

    // update mask to indicate valid output points.
    // an output point is valid when
    //   - corresponding input pixel is valid (input mask is valid)
    //   - corresponding input pixel is in the image frame
    for (int x = 0; x < mask.size().width; ++x) {
      for (int y = 0; y < mask.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Point2f &s(src.at< cv::Point2f >(y, x));
        m = (m != 0 && frame_.contains(s)) ? 1 : 0;
      }
    }
  }

  virtual void fromCameraInfo(const sensor_msgs::CameraInfo &camera_info) {
    // TODO: implement!!
  }

  virtual sensor_msgs::CameraInfoPtr toCameraInfo() const {
    // TODO: implement!!
  }

private:
  cv::Matx33d camera_matrix_;
  std::vector< double > dist_coeffs_;
  cv::Rect_< float > frame_;
};

} // namespace image_reprojection_plugins

#endif /* IMAGE_REPROJECTION_PLUGINS_PINHOLE_CAMERA_MODEL_HPP */
