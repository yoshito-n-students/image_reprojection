#include <gtest/gtest.h>

#include <image_reprojection_plugins/MeshStamped.h>
#include <image_reprojection_plugins/dem_surface_model.hpp>
#include <image_reprojection_plugins/mesh_surface_model.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>

namespace irp = image_reprojection_plugins;

// the global random number generator
cv::RNG g_rng(std::time(NULL));

void twoRandomPoints(geometry_msgs::Point &point1, geometry_msgs::Point &point2) {
  point1.x = g_rng.uniform(-1., 1.);
  point1.y = g_rng.uniform(-1., 1.);
  point1.z = g_rng.uniform(-1., 1.);

  cv::Vec3d dp;
  dp[0] = g_rng.uniform(-1., 1.);
  dp[1] = g_rng.uniform(-1., 1.);
  dp[2] = g_rng.uniform(-1., 1.);
  cv::normalize(dp);

  point2.x = point1.x + dp[0];
  point2.y = point1.y + dp[1];
  point2.z = point1.z + dp[2];
}

geometry_msgs::Quaternion randomOrientation() {
  // random rotaion axis
  cv::Vec3d axis;
  axis[0] = g_rng.uniform(-1., 1.);
  axis[1] = g_rng.uniform(-1., 1.);
  axis[2] = g_rng.uniform(-1., 1.);
  cv::normalize(axis, axis);

  // random rotation angle
  const double half_angle(g_rng.uniform(-M_PI / 2., M_PI / 2.));

  // quaternion from rotation axis and angle
  geometry_msgs::Quaternion orientation;
  const double sin(std::sin(half_angle));
  orientation.w = std::cos(half_angle);
  orientation.x = axis[0] * sin;
  orientation.y = axis[1] * sin;
  orientation.z = axis[2] * sin;

  return orientation;
}

cv::Mat randomDirection(const int width, const int height) {
  cv::Mat direction(width, height, CV_32FC3);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      cv::Vec3f &d(direction.at< cv::Vec3f >(y, x));
      d[0] = g_rng.uniform(-1., 1.);
      d[1] = g_rng.uniform(-1., 1.);
      d[2] = g_rng.uniform(-1., 1.);
    }
  }
  return direction;
}

// geometry_msgs::Point from transform matrix and source coordinates
geometry_msgs::Point transformPoint(const tf::Transform &transform, const double x, const double y,
                                    const double z) {
  const tf::Vector3 tf_point(transform(tf::Vector3(x, y, z)));
  geometry_msgs::Point ros_point;
  ros_point.x = tf_point[0];
  ros_point.y = tf_point[1];
  ros_point.z = tf_point[2];
  return ros_point;
}

// 1-line construction of shape_msgs::MeshTriangle
shape_msgs::MeshTriangle meshTriangle(const unsigned int id0, const unsigned int id1,
                                      const unsigned int id2) {
  shape_msgs::MeshTriangle triangle;
  triangle.vertex_indices[0] = id0;
  triangle.vertex_indices[0] = id1;
  triangle.vertex_indices[0] = id2;
  return triangle;
}

// convert DEM to mesh, assuming DEM size is (1, 1)
irp::MeshStamped toMesh(const nav_msgs::OccupancyGrid &grid) {
  irp::MeshStamped mesh;
  mesh.header = grid.header;

  tf::Transform dem2tf;
  tf::poseMsgToTF(grid.info.origin, dem2tf);

  const double lxy(grid.info.resolution);
  const double lz(grid.data[0] / 100.);
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, 0., 0., 0.));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, lxy, 0., 0.));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, lxy, lxy, 0.));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, 0., lxy, 0.));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, 0., 0., lz));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, lxy, 0., lz));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, lxy, lxy, lz));
  mesh.mesh.vertices.push_back(transformPoint(dem2tf, 0., lxy, lz));

  // bottom
  mesh.mesh.triangles.push_back(meshTriangle(0, 1, 2));
  mesh.mesh.triangles.push_back(meshTriangle(0, 2, 3));
  // top
  mesh.mesh.triangles.push_back(meshTriangle(4, 5, 6));
  mesh.mesh.triangles.push_back(meshTriangle(4, 6, 7));
  // front
  mesh.mesh.triangles.push_back(meshTriangle(0, 3, 7));
  mesh.mesh.triangles.push_back(meshTriangle(0, 4, 7));
  // back
  mesh.mesh.triangles.push_back(meshTriangle(1, 2, 6));
  mesh.mesh.triangles.push_back(meshTriangle(1, 5, 6));
  // left
  mesh.mesh.triangles.push_back(meshTriangle(0, 1, 5));
  mesh.mesh.triangles.push_back(meshTriangle(0, 4, 5));
  // right
  mesh.mesh.triangles.push_back(meshTriangle(2, 3, 7));
  mesh.mesh.triangles.push_back(meshTriangle(2, 6, 7));

  return mesh;
}

TEST(DEMSurfaceModel, compare10000) {
  //
  geometry_msgs::Point ray_origin, dem_origin;
  twoRandomPoints(ray_origin, dem_origin);

  //
  nav_msgs::OccupancyGrid dem;
  dem.info.resolution = 0.5;
  dem.info.width = 1;
  dem.info.height = 1;
  dem.info.origin.position = dem_origin;
  dem.info.origin.orientation = randomOrientation();
  dem.data.resize(1, 100);

  //
  irp::DEMSurfaceModel dem_model;
  dem_model.init("dem", ros::M_string(), ros::V_string());
  dem_model.update(dem);

  // initialize tested model with random mesh
  irp::MeshSurfaceModel mesh_model;
  mesh_model.init("mesh", ros::M_string(), ros::V_string());
  mesh_model.update(toMesh(dem));

  //
  const int width(100), height(100);
  const cv::Mat ray_direction(randomDirection(width, height));

  // generate truth data at random
  cv::Mat dem_intersection, dem_mask(cv::Mat::ones(ray_direction.size(), CV_8UC1));
  dem_model.intersection(cv::Vec3f(ray_origin.x, ray_origin.y, ray_origin.z), ray_direction,
                         dem_intersection, dem_mask);


  cv::Mat mesh_intersection, mesh_mask(cv::Mat::ones(ray_direction.size(), CV_8UC1));
  mesh_model.intersection(cv::Vec3f(ray_origin.x, ray_origin.y, ray_origin.z), ray_direction,
                          mesh_intersection, mesh_mask);

  // compare results from tested model and expected results
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      //
      const unsigned char dm(dem_mask.at< unsigned char >(y, x));
      const unsigned char mm(mesh_mask.at< unsigned char >(y, x));
      EXPECT_TRUE(dm == mm);
      // compare intersection points
      if (dm != 0 && mm != 0) {
        const cv::Vec3f di(dem_intersection.at< cv::Vec3f >(y, x));
        const cv::Vec3f mi(mesh_intersection.at< cv::Vec3f >(y, x));
        EXPECT_NEAR(di[0], mi[0], 0.001);
        EXPECT_NEAR(di[1], mi[1], 0.001);
        EXPECT_NEAR(di[2], mi[2], 0.001);
      }
    }
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_dem_surface_model");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}