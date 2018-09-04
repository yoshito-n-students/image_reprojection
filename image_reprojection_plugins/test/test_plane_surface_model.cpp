#include <gtest/gtest.h>

#include <image_reprojection_plugins/PlaneStamped.h>
#include <image_reprojection_plugins/plane_surface_model.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <opencv2/core/core.hpp>

namespace irp = image_reprojection_plugins;

// the global random number generator
cv::RNG g_rng(std::time(NULL));

cv::Vec3f randomPoint() {
  return cv::Vec3f(g_rng.uniform(-1., 1.), g_rng.uniform(-1., 1.), g_rng.uniform(-1., 1.));
}

cv::Mat randomPoints(const cv::Size &size) {
  cv::Mat points(size, CV_32FC3);
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      points.at< cv::Vec3f >(y, x) = randomPoint();
    }
  }
  return points;
}

irp::PlaneStamped toPlane(const cv::Vec3f &point, const cv::Vec3f &normal) {
  irp::PlaneStamped plane;
  plane.point.x = point[0];
  plane.point.y = point[1];
  plane.point.z = point[2];
  plane.normal.x = normal[0];
  plane.normal.y = normal[1];
  plane.normal.z = normal[2];
  return plane;
}

TEST(PlaneSurfaceModel, randomIntersection) {
  // initialize tested model with random plane
  irp::PlaneSurfaceModel model;
  model.init("plane", ros::M_string(), ros::V_string());
  const cv::Vec3f point(randomPoint()), normal(randomPoint());
  model.update(toPlane(point, normal));

  // calculate intersections using tested model
  const cv::Size size(100, 100);
  const cv::Vec3f ray_origin(randomPoint());
  const cv::Mat ray_direction(randomPoints(size));
  cv::Mat intersection, mask(cv::Mat::ones(size, CV_8UC1));
  model.intersection(ray_origin, ray_direction, intersection, mask);

  // check intersections
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      if (mask.at< unsigned char >(y, x) != 0) {
        // intersection point is on the plane
        const cv::Vec3f i(intersection.at< cv::Vec3f >(y, x));
        EXPECT_NEAR(normal.dot(intersection.at< cv::Vec3f >(y, x) - point), 0., 0.001);

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