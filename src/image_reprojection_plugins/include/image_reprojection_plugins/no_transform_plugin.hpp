#ifndef _IMAGE_REPROJECTION_PLUGINS_NO_TRANSFORM_PLUGIN_HPP_
#define _IMAGE_REPROJECTION_PLUGINS_NO_TRANSFORM_PLUGIN_HPP_

#include <opencv2/core/core.hpp>

#include <image_reprojection/transform_plugin.hpp>
#include <ros/console.h>

namespace image_reprojection_plugins {

class NoTransformPlugin : public image_reprojection::TransformPlugin {
   public:
    NoTransformPlugin() {}

    virtual ~NoTransformPlugin() {}

   private:
    virtual void onInit() { ROS_INFO_STREAM(getName() << " has been initialized"); }

    virtual void onTransform(const cv::Mat& src, cv::Mat& dst) { dst = src.clone(); }

    virtual void onInverseTransform(const cv::Mat& src, cv::Mat& dst) { dst = src.clone(); }
};
}

#endif /* _IMAGE_REPROJECTION_PLUGINS_NO_TRANSFORM_PLUGIN_HPP_ */
