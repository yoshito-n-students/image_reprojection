#ifndef IMAGE_REPROJECTION_PLUGINS_DUAL_FISHEYE_SPLITTER_HPP
#define IMAGE_REPROJECTION_PLUGINS_DUAL_FISHEYE_SPLITTER_HPP

#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <boost/scoped_ptr.hpp>

namespace image_reprojection_plugins {

class DualFisheyeSplitter : public nodelet::Nodelet {
public:
private:
  virtual void onInit() {
    // node handles
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    // setup left-hand camera things
    {
      ros::NodeHandle left_nh(nh, "left");
      ros::NodeHandle left_pnh(pnh, "left");
      image_transport::ImageTransport left_it(left_nh);

      left_frame_id_ = left_pnh.param<std::string>("frame_id", "front_camera");
      left_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          left_nh, left_pnh.param<std::string>("camera_name", "front_camera"),
          left_pnh.param<std::string>("camera_info_url", "")));
      left_publisher_ = left_it.advertiseCamera("image", 1, true);
    }

    // setup right-hand camera things
    {
      ros::NodeHandle right_nh(nh, "right");
      ros::NodeHandle right_pnh(pnh, "right");
      image_transport::ImageTransport right_it(right_nh);

      right_frame_id_ = right_pnh.param<std::string>("frame_id", "back_camera");
      right_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          right_nh, right_pnh.param<std::string>("camera_name", "back_camera"),
          right_pnh.param<std::string>("camera_info_url", "")));
      right_publisher_ = right_it.advertiseCamera("image", 1, true);
    }

    // setup dual-fisheye image subscriber
    const image_transport::TransportHints default_hints;
    subscriber_ = it.subscribe("image", 1, &DualFisheyeSplitter::split, this,
                               image_transport::TransportHints(default_hints.getTransport(),
                                                               default_hints.getRosHints(), pnh));
  }

  void split(const sensor_msgs::ImageConstPtr &image_msg) {
    try {
      // dual image
      const cv_bridge::CvImageConstPtr image(cv_bridge::toCvShare(image_msg));

      // publish left hand image with camera info
      {
        cv_bridge::CvImage left_image;
        left_image.header.stamp = image->header.stamp;
        left_image.header.frame_id = left_frame_id_;
        left_image.encoding = image->encoding;
        left_image.image = image->image.colRange(0, image->image.cols / 2);

        const sensor_msgs::CameraInfoPtr left_info(
            boost::make_shared<sensor_msgs::CameraInfo>(left_info_manager_->getCameraInfo()));
        left_info->header.stamp = image->header.stamp;
        left_info->header.frame_id = left_frame_id_;

        left_publisher_.publish(left_image.toImageMsg(), left_info);
      }

      // publish right hand image with camera info
      {
        cv_bridge::CvImage right_image;
        right_image.header.stamp = image->header.stamp;
        right_image.header.frame_id = right_frame_id_;
        right_image.encoding = image->encoding;
        right_image.image = image->image.colRange(image->image.cols / 2, image->image.cols);

        const sensor_msgs::CameraInfoPtr right_info(
            boost::make_shared<sensor_msgs::CameraInfo>(right_info_manager_->getCameraInfo()));
        right_info->header.stamp = image->header.stamp;
        right_info->header.frame_id = right_frame_id_;

        right_publisher_.publish(right_image.toImageMsg(), right_info);
      }
    } catch (const std::exception &ex) {
      NODELET_ERROR_STREAM(ex.what());
    }
  }

private:
  std::string left_frame_id_, right_frame_id_;

  image_transport::Subscriber subscriber_;
  image_transport::CameraPublisher left_publisher_, right_publisher_;
  boost::scoped_ptr<camera_info_manager::CameraInfoManager> left_info_manager_, right_info_manager_;
};

} // namespace image_reprojection_plugins

#endif