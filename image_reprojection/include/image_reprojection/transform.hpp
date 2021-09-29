#ifndef IMAGE_REPROJECTION_TRAMSFORM_HPP
#define IMAGE_REPROJECTION_TRAMSFORM_HPP

#include <tf/tf.h>

#include <opencv2/core/core.hpp>

namespace image_reprojection {

static inline cv::Vec3f transform(const cv::Vec3f &cv_vec, const tf::Transform &tf_transform) {
  const tf::Vector3 tf_vec = tf_transform(tf::Vector3(cv_vec[0], cv_vec[1], cv_vec[2]));
  return cv::Vec3f(tf_vec[0], tf_vec[1], tf_vec[2]);
}

static inline cv::Vec3f transform(const cv::Vec3f &cv_vec, const tf::Matrix3x3 &tf_basis) {
  const tf::Vector3 tf_vec = tf_basis * tf::Vector3(cv_vec[0], cv_vec[1], cv_vec[2]);
  return cv::Vec3f(tf_vec[0], tf_vec[1], tf_vec[2]);
}

template <typename TransformInfo>
cv::Mat transform(const cv::Mat &src, const TransformInfo &info, const cv::Mat &mask) {
  CV_Assert(src.type() == CV_32FC3);
  CV_Assert(mask.type() == CV_8UC1);
  CV_Assert(src.size() == mask.size());

  cv::Mat dst(src.size(), src.type());
  for (int x = 0; x < src.size().width; ++x) {
    for (int y = 0; y < src.size().height; ++y) {
      if (mask.at<unsigned char>(y, x) != 0) {
        dst.at<cv::Vec3f>(y, x) = transform(src.at<cv::Vec3f>(y, x), info);
      }
    }
  }
  return dst;
}

} // namespace image_reprojection

#endif