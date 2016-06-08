#ifndef IMAGE_REPROJECTION_PLUGINS_TF_TRANSFORM_HPP
#define IMAGE_REPROJECTION_PLUGINS_TF_TRANSFORM_HPP

#include <string>

#include <image_reprojection/transform_interface.hpp>
#include <image_reprojection_plugins/transform_helper.hpp>
#include <param_utilities/param_utilities.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_listener.h>

namespace image_reprojection_plugins {

class TFTransform : public image_reprojection::TransformInterface {
   public:
    TFTransform() {}

    virtual ~TFTransform() {}

   private:
    virtual void onInit() {
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        frame_id_src_ = param_utilities::param<std::string>(pnh, "frame_id_src", "world");
        frame_id_dst_ = param_utilities::param<std::string>(pnh, "frame_id_dst", "dst");

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onTransform(const cv::Mat& src, cv::Mat& dst) {
        setTransform();
        helper_.transform(src, dst);
    }

    virtual void onInverseTransform(const cv::Mat& src, cv::Mat& dst) {
        setTransform();
        helper_.inverseTransform(src, dst);
    }

    void setTransform() {
        // get the latest transform and set it to the transformer
        tf::StampedTransform transform;
        listener_.lookupTransform(frame_id_dst_, frame_id_src_, ros::Time(0.), transform);
        helper_.set(transform);
    }

   private:
    // parameters
    std::string frame_id_src_;
    std::string frame_id_dst_;

    // worker objects
    tf::TransformListener listener_;
    TransformHelper helper_;
};
}

#endif  // IMAGE_REPROJECTION_PLUGINS_TF_TRANSFORM_HPP