#include <string>
#include <vector>

#include <image_reprojection_plugins/MeshStamped.h>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/time.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

shape_msgs::MeshTriangle toMeshTriangle(const XmlRpc::XmlRpcValue &src) {
  if (src.size() != 3) {
    throw XmlRpc::XmlRpcException("number of indices (must be 3)");
  }
  shape_msgs::MeshTriangle dst;
  dst.vertex_indices[0] = static_cast< int >(XmlRpc::XmlRpcValue(src[0]));
  dst.vertex_indices[1] = static_cast< int >(XmlRpc::XmlRpcValue(src[1]));
  dst.vertex_indices[2] = static_cast< int >(XmlRpc::XmlRpcValue(src[2]));
  return dst;
}

std::vector< shape_msgs::MeshTriangle > toMeshTriangles(const XmlRpc::XmlRpcValue &src) {
  std::vector< shape_msgs::MeshTriangle > dst;
  for (int i = 0; i < src.size(); ++i) {
    dst.push_back(toMeshTriangle(src[i]));
  }
  return dst;
}

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

std::vector< geometry_msgs::Point > toPoints(const XmlRpc::XmlRpcValue &src) {
  std::vector< geometry_msgs::Point > dst;
  for (int i = 0; i < src.size(); ++i) {
    dst.push_back(toPoint(src[i]));
  }
  return dst;
}

int main(int argc, char *argv[]) {
  namespace irp = image_reprojection_plugins;

  ros::init(argc, argv, "static_mesh_publisher");
  ros::NodeHandle nh, pnh("~");

  irp::MeshStamped mesh;
  mesh.header.frame_id = pnh.param< std::string >("frame_id", "");
  mesh.mesh.triangles = toMeshTriangles(pnh.param("triangles", XmlRpc::XmlRpcValue()));
  mesh.mesh.vertices = toPoints(pnh.param("vertices", XmlRpc::XmlRpcValue()));
  ROS_INFO_STREAM(mesh);

  ros::Publisher publisher;
  publisher = nh.advertise< irp::MeshStamped >("mesh", 1, true);

  ros::Rate rate(pnh.param("rate", 10.));
  while (ros::ok()) {
    mesh.header.stamp = ros::Time::now();
    publisher.publish(mesh);

    rate.sleep();
  }

  return 0;
}