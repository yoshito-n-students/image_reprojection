#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/rate.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "tf_tracking_gimbal");
  ros::NodeHandle nh;

  const std::string frame_id(ros::param::param<std::string>("~frame_id", ""));
  const std::string parent_frame_id(ros::param::param<std::string>("~parent_frame_id", ""));
  const std::string tracked_frame_id(ros::param::param<std::string>("~tracked_frame_id", ""));
  ros::Rate rate(ros::param::param("~frequency", 10.));

  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  while (ros::ok()) {
    try {
      tf::StampedTransform tracked2parent;
      tf_listener.lookupTransform(parent_frame_id, tracked_frame_id, ros::Time(0), tracked2parent);

      const tf::Vector3 p(tracked2parent(tf::Vector3(0., 0., 0.)));
      const tf::Quaternion dst2tmp(tf::Vector3(-1., 0., 0.),
                                   std::atan2(p.y(), std::sqrt(p.x() * p.x() + p.z() * p.z())));
      const tf::Quaternion tmp2parent(tf::Vector3(0., 1., 0.), std::atan2(p.x(), p.z()));

      const tf::StampedTransform dst2parent(tf::Transform(tmp2parent * dst2tmp),
                                            tracked2parent.stamp_, parent_frame_id, frame_id);
      tf_broadcaster.sendTransform(dst2parent);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM_THROTTLE(1., ex.what());
    }

    rate.sleep();
  }

  return 0;
}