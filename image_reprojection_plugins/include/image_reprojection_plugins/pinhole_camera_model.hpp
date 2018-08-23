#ifndef IMAGE_REPROJECTION_PLUGINS_PINHOLE_CAMERA_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_PINHOLE_CAMERA_MODEL_HPP

#include <algorithm>
#include <vector>

#include <image_reprojection/camera_model.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/make_shared.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace image_reprojection_plugins {

class PinholeCameraModel : public image_reprojection::CameraModel {
public:
  PinholeCameraModel() {}

  virtual ~PinholeCameraModel() {}

  virtual void fromCameraInfo(const sensor_msgs::CameraInfo &camera_info) {
    // assert distortion type and number of distortion parameters
    CV_Assert(camera_info.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB ||
              camera_info.distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL);
    CV_Assert(camera_info.D.size() == 0 || camera_info.D.size() == 4 || camera_info.D.size() == 5 ||
              camera_info.D.size() == 8 || camera_info.D.size() == 12 ||
              camera_info.D.size() == 14);

    boost::unique_lock< boost::shared_mutex > write_lock(mutex_);

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
    dist_coeffs_ = camera_info.D;
  }

  virtual sensor_msgs::CameraInfoPtr toCameraInfo() const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);
    return boost::make_shared< sensor_msgs::CameraInfo >(camera_info_);
  }

private:
  virtual void onInit() {}

  virtual void onProject3dToPixel(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);

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
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);

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

private:
  mutable boost::shared_mutex mutex_;
  sensor_msgs::CameraInfo camera_info_;
  cv::Matx33d camera_matrix_;
  std::vector< double > dist_coeffs_;
  cv::Rect_< float > frame_;
};

} // namespace image_reprojection_plugins

#endif /* IMAGE_REPROJECTION_PLUGINS_PINHOLE_CAMERA_MODEL_HPP */
