#include <string>

#include <image_reprojection_plugins/SphereStamped.h>
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

  ros::init(argc, argv, "static_sphere_publisher");
  ros::NodeHandle nh, pnh("~");

  irp::SphereStamped sphere;
  sphere.header.frame_id = pnh.param<std::string>("frame_id", "");
  sphere.center = toPoint(pnh.param("center", XmlRpc::XmlRpcValue()));
  sphere.radius = pnh.param("radius", 1.);
  ROS_INFO_STREAM(sphere);

  ros::Publisher publisher;
  publisher = nh.advertise<irp::SphereStamped>("sphere", 1, true);

  ros::Rate rate(pnh.param("rate", 10.));
  while (ros::ok()) {
    sphere.header.stamp = ros::Time::now();
    publisher.publish(sphere);

    rate.sleep();
  }

  return 0;
}