#ifndef IMAGE_REPROJECTION_PLUGINS_TF_TRANSFORM_HPP
#define IMAGE_REPROJECTION_PLUGINS_TF_TRANSFORM_HPP

#include <string>

#include <image_reprojection/transform_interface.hpp>
#include <image_reprojection_plugins/transform_helper.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <utility_headers/param.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class TFTransform : public image_reprojection::TransformInterface {
   public:
    TFTransform() {}

    virtual ~TFTransform() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        frame_id_src_ = uhp::param<std::string>(pnh, "frame_id_src", "world");
        frame_id_dst_ = uhp::param<std::string>(pnh, "frame_id_dst", "dst");

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
        // get the latest tf transform
        tf::StampedTransform tf_transform;
        listener_.lookupTransform(frame_id_dst_, frame_id_src_, ros::Time(0.), tf_transform);

        // tf transform -> opencv transform
        cv::Matx44d cv_transform(cv::Matx44d::eye());
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cv_transform(i, j) = tf_transform.getBasis()[i][j];
            }
            cv_transform(i, 3) = tf_transform.getOrigin()[i];
        }

        // set the opencv transform
        helper_.set(cv_transform);
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