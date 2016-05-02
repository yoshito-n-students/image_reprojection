#ifndef _IMAGE_REPROJECTION_PLUGINS_ROTATION_TRANSFORM_PLUGIN_HPP_
#define _IMAGE_REPROJECTION_PLUGINS_ROTATION_TRANSFORM_PLUGIN_HPP_

#include <opencv2/core/core.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <geometry_msgs/Quaternion.h>
#include <image_reprojection/transform_plugin.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/service_server.h>
#include <tf/transform_datatypes.h>
#include <utility_headers/param.hpp>

namespace image_reprojection_plugins {

class RotationTransformPlugin : public image_reprojection::TransformPlugin {
   public:
    RotationTransformPlugin()
        : rotation_home_(cv::Mat::eye(3, 3, CV_64FC1)),
          rotation_(cv::Mat::eye(3, 3, CV_64FC1)),
          rotation_inv_(cv::Mat::eye(3, 3, CV_64FC1)) {}

    virtual ~RotationTransformPlugin() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        const ros::NodeHandle& nh(getNodeHandle());
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        {
            boost::array<double, 3> rpy;
            uhp::get(pnh, "home_rpy", rpy);
            tf::Quaternion qtn;
            qtn.setRPY(rpy[0], rpy[1], rpy[2]);
            const tf::Matrix3x3 mat(qtn);
            {
                boost::lock_guard<boost::mutex> lock(mutex_);
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        rotation_home_.at<double>(i, j) = mat[i][j];
                    }
                }
                rotation_ = rotation_home_.clone();
                rotation_inv_ = rotation_home_.inv();
            }
        }

        // server_ = nh.advertiseService<>
        //	("set_rotation",1,&RotationTransformPlugin::setRotation,this);

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onTransform(const cv::Mat& src, cv::Mat& dst) {
        cv::Mat rotation;
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            rotation = rotation_.clone();
        }
        dst = src.reshape(1, src.total()) * rotation.t();
        dst = dst.reshape(3, src.size().height);
    }

    virtual void onInverseTransform(const cv::Mat& src, cv::Mat& dst) {
        cv::Mat rotation_inv;
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            rotation_inv = rotation_inv_.clone();
        }
        dst = src.reshape(1, src.total()) * rotation_inv.t();
        dst = dst.reshape(3, src.size().height);
    }

    /*
    void setRotation(const ){

    }
    */

    static cv::Mat getRotationMatrix(const double x, const double y, const double z,
                                     const double w) {
        const tf::Quaternion tf_qtn(x, y, z, w);
        const tf::Matrix3x3 tf_mat(tf_qtn);
        cv::Mat cv_mat(3, 3, CV_64FC1);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cv_mat.at<double>(i, j) = tf_mat[i][j];
            }
        }
        return cv_mat;
    }

   private:
    ros::ServiceServer server_;
    cv::Mat rotation_home_;
    cv::Mat rotation_;
    cv::Mat rotation_inv_;
    boost::mutex mutex_;
};
}

#endif /* _IMAGE_REPROJECTION_PLUGINS_ROTATION_TRANSFORM_PLUGIN_HPP_ */
