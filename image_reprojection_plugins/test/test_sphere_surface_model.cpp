#include <gtest/gtest.h>

#include <image_reprojection_plugins/SphereStamped.h>
#include <image_reprojection_plugins/sphere_surface_model.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <opencv2/core/core.hpp>

#include "random.hpp"

namespace irp = image_reprojection_plugins;

irp::SphereStamped toSphereMsg(const cv::Vec3f &center, const double radius) {
  irp::SphereStamped sphere;
  sphere.center = toPointMsg(center);
  sphere.radius = radius;
  return sphere;
}

TEST(SphereSurfaceModel, randomIntersection) {
  // initialize tested model with random sphere
  irp::SphereSurfaceModel model;
  model.init("sphere", ros::M_string(), ros::V_string());
  const cv::Vec3f center(randomPoint(-1., 1.));
  const double radius(randomNonZeroValue(0., 1.));
  model.update(toSphereMsg(center, radius));

  // calculate intersections using tested model
  const cv::Size size(100, 100);
  const cv::Vec3f ray_origin(randomPoint(-1., 1.));
  const cv::Mat ray_direction(randomNonZeroPoints(size, -1., 1.));
  cv::Mat intersection, mask(cv::Mat::ones(size, CV_8UC1));
  model.intersection(ray_origin, ray_direction, intersection, mask);

  // check intersections
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      if (mask.at< unsigned char >(y, x) != 0) {
        // intersection point is on the sphere
        const cv::Vec3f i(intersection.at< cv::Vec3f >(y, x));
        EXPECT_NEAR(cv::norm(i, center), radius, 0.001 * radius);

        // ray direction vector points intersection point
        const cv::Vec3f d(ray_direction.at< cv::Vec3f >(y, x));
        const double t((i - ray_origin).dot(d) / d.dot(d));
        EXPECT_TRUE(t >= 0.);
        EXPECT_NEAR(cv::norm(i, ray_origin + t * d), 0., 0.001 * cv::norm(t * d));
      } else {
        // TODO: check no intersection
      }
    }
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_mesh_surface_model");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}