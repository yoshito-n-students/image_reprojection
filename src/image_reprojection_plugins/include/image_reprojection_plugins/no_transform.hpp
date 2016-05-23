#ifndef IMAGE_REPROJECTION_PLUGINS_NO_TRANSFORM_HPP
#define IMAGE_REPROJECTION_PLUGINS_NO_TRANSFORM_HPP

#include <image_reprojection/transform_interface.hpp>
#include <ros/console.h>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class NoTransform : public image_reprojection::TransformInterface {
   public:
    NoTransform() {}

    virtual ~NoTransform() {}

   private:
    virtual void onInit() { ROS_INFO_STREAM(getName() << " has been initialized"); }

    virtual void onTransform(const cv::Mat& src, cv::Mat& dst) { dst = src.clone(); }

    virtual void onInverseTransform(const cv::Mat& src, cv::Mat& dst) { dst = src.clone(); }
};
}

#endif /* IMAGE_REPROJECTION_PLUGINS_NO_TRANSFORM_HPP */
