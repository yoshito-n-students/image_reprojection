#include <gtest/gtest.h>

#include <image_reprojection_plugins/MeshStamped.h>
#include <image_reprojection_plugins/dem_surface_model.hpp>
#include <image_reprojection_plugins/mesh_surface_model.hpp>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>

#include "random.hpp"

namespace irp = image_reprojection_plugins;

geometry_msgs::Quaternion randomOrientationMsg() {
  // random rotaion axis and angle
  const cv::Vec3d axis = cv::normalize(randomNonZeroPoint(-1., 1.));
  const double half_angle = randomValue(-M_PI / 2., M_PI / 2.);

  // quaternion from rotation axis and angle
  geometry_msgs::Quaternion orientation;
  const double sin = std::sin(half_angle);
  orientation.w = std::cos(half_angle);
  orientation.x = axis[0] * sin;
  orientation.y = axis[1] * sin;
  orientation.z = axis[2] * sin;

  return orientation;
}

// geometry_msgs::Point from transform matrix and source coordinates
geometry_msgs::Point transformedPointMsg(const double x, const double y, const double z,
                                         const tf::Transform &transform) {
  const tf::Vector3 tf_point = transform(tf::Vector3(x, y, z));
  geometry_msgs::Point ros_point;
  ros_point.x = tf_point[0];
  ros_point.y = tf_point[1];
  ros_point.z = tf_point[2];
  return ros_point;
}

// 1-line construction of shape_msgs::MeshTriangle
shape_msgs::MeshTriangle meshTriangleMsg(const unsigned int id0, const unsigned int id1,
                                         const unsigned int id2) {
  shape_msgs::MeshTriangle triangle;
  triangle.vertex_indices = {id0, id1, id2};
  return triangle;
}

// convert DEM to mesh, assuming DEM size is (1, 1)
irp::MeshStamped toMeshMsg(const nav_msgs::OccupancyGrid &grid) {
  irp::MeshStamped mesh;
  mesh.header = grid.header;

  tf::Transform dem2tf;
  tf::poseMsgToTF(grid.info.origin, dem2tf);

  const double lxy = grid.info.resolution;
  const double lz = grid.data[0] / 100.;
  mesh.mesh.vertices = {
      transformedPointMsg(0., 0., 0., dem2tf),   transformedPointMsg(lxy, 0., 0., dem2tf),
      transformedPointMsg(lxy, lxy, 0., dem2tf), transformedPointMsg(0., lxy, 0., dem2tf),
      transformedPointMsg(0., 0., lz, dem2tf),   transformedPointMsg(lxy, 0., lz, dem2tf),
      transformedPointMsg(lxy, lxy, lz, dem2tf), transformedPointMsg(0., lxy, lz, dem2tf)};
  mesh.mesh.triangles = {meshTriangleMsg(0, 1, 2), meshTriangleMsg(0, 2, 3),  // bottom
                         meshTriangleMsg(4, 5, 6), meshTriangleMsg(4, 6, 7),  // top
                         meshTriangleMsg(0, 3, 7), meshTriangleMsg(0, 4, 7),  // front
                         meshTriangleMsg(1, 2, 6), meshTriangleMsg(1, 5, 6),  // back
                         meshTriangleMsg(0, 1, 5), meshTriangleMsg(0, 4, 5),  // left
                         meshTriangleMsg(2, 3, 7), meshTriangleMsg(2, 6, 7)}; // right

  return mesh;
}

TEST(DEMSurfaceModel, compareMeshIntersection) {
  // 1-box DEM at random pose
  nav_msgs::OccupancyGrid dem;
  const cv::Vec3f dem_origin = randomPoint(-1., 1.);
  dem.info.resolution = 0.5;
  dem.info.width = 1;
  dem.info.height = 1;
  dem.info.origin.position = toPointMsg(dem_origin);
  dem.info.origin.orientation = randomOrientationMsg();
  dem.data.resize(1, 100);

  // initialize tested model with random DEM
  irp::DEMSurfaceModel dem_model;
  dem_model.init("dem", ros::M_string(), ros::V_string());
  dem_model.update(dem);

  // initialize comparative model with a mesh generated from the same DEM
  irp::MeshSurfaceModel mesh_model;
  mesh_model.init("mesh", ros::M_string(), ros::V_string());
  mesh_model.update(toMeshMsg(dem));

  // random ray whose source is apart 1.0 from DEM origin and has random direction
  const cv::Size size(100, 100);
  const cv::Vec3f ray_origin = dem_origin + cv::normalize(randomNonZeroPoint(-1, 1.));
  const cv::Mat ray_direction = randomNonZeroPoints(size, -1., 1.);

  // calculate intersection using tested model
  cv::Mat dem_intersection, dem_mask = cv::Mat::ones(ray_direction.size(), CV_8UC1);
  dem_model.intersection(ray_origin, ray_direction, dem_intersection, dem_mask);
  EXPECT_GT(cv::countNonZero(dem_mask), 0);

  // calculate intersection using comparative model
  cv::Mat mesh_intersection, mesh_mask = cv::Mat::ones(ray_direction.size(), CV_8UC1);
  mesh_model.intersection(ray_origin, ray_direction, mesh_intersection, mesh_mask);
  EXPECT_GT(cv::countNonZero(mesh_mask), 0);

  // compare results from tested and comparative models
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      // compare mask values
      const unsigned char dm = dem_mask.at<unsigned char>(y, x);
      const unsigned char mm = mesh_mask.at<unsigned char>(y, x);
      EXPECT_TRUE((dm != 0 && mm != 0) || (dm == 0 && mm == 0));
      // compare intersection points
      if (dm != 0 && mm != 0) {
        const cv::Vec3f &di = dem_intersection.at<cv::Vec3f>(y, x);
        const cv::Vec3f &mi = mesh_intersection.at<cv::Vec3f>(y, x);
        EXPECT_NEAR(cv::norm(di, mi), 0., 0.001 * cv::norm(mi));
      }
    }
  }
}