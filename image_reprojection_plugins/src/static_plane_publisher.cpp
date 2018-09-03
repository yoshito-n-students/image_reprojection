#include <string>

#include <image_reprojection_plugins/PlaneStamped.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <boost/array.hpp>

geometry_msgs::Point toPoint(const XmlRpc::XmlRpcValue &src) {
  if (src.size() != 3) {
    throw XmlRpc::XmlRpcException("number of coordinates (must be 3)");
  }
  geometry_msgs::Point dst;
  dst.x = XmlRpc::XmlRpcValue(src[0]);
  dst.y = XmlRpc::XmlRpcValue(src[1]);
  dst.z = XmlRpc::XmlRpcValue(src[2]);
  return dst;
}

int main(int argc, char *argv[]) {
  namespace irp = image_reprojection_plugins;

  ros::init(argc, argv, "static_plane_publisher");
  ros::NodeHandle nh, pnh("~");

  irp::PlaneStamped plane;
  plane.header.frame_id = pnh.param< std::string >("frame_id", "");
  plane.point = toPoint(pnh.param("point", XmlRpc::XmlRpcValue()));
  plane.normal = toPoint(pnh.param("normal", XmlRpc::XmlRpcValue()));
  ROS_INFO_STREAM(plane);

  ros::Publisher publisher;
  publisher = nh.advertise< irp::PlaneStamped >("plane", 1, true);

  ros::Rate rate(pnh.param("rate", 10.));
  while (ros::ok()) {
    plane.header.stamp = ros::Time::now();
    publisher.publish(plane);

    rate.sleep();
  }

  return 0;
}