#include <gtest/gtest.h>

#include <image_reprojection_plugins/PlaneStamped.h>
#include <image_reprojection_plugins/plane_surface_model.hpp>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <opencv2/core/core.hpp>

#include "random.hpp"

namespace irp = image_reprojection_plugins;

irp::PlaneStamped toPlaneMsg(const cv::Vec3f &point, const cv::Vec3f &normal) {
  irp::PlaneStamped plane;
  plane.point = toPointMsg(point);
  plane.normal = toPointMsg(normal);
  return plane;
}

TEST(PlaneSurfaceModel, randomIntersection) {
  // initialize tested model with random plane
  irp::PlaneSurfaceModel model;
  model.init("plane", ros::M_string(), ros::V_string());
  const cv::Vec3f point(randomPoint(-1., 1.)), normal(randomNonZeroPoint(-1., 1.));
  model.update(toPlaneMsg(point, normal));

  // calculate intersections using tested model
  const cv::Size size(100, 100);
  const cv::Vec3f ray_origin(randomPoint(-1., 1.));
  const cv::Mat ray_direction(randomNonZeroPoints(size, -1., 1.));
  cv::Mat intersection, mask(cv::Mat::ones(size, CV_8UC1));
  model.intersection(ray_origin, ray_direction, intersection, mask);

  // check intersections
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      if (mask.at<unsigned char>(y, x) != 0) {
        // intersection point is on the plane
        const cv::Vec3f i(intersection.at<cv::Vec3f>(y, x));
        EXPECT_NEAR(normal.dot(i - point), 0., 0.001 * cv::norm(normal) * cv::norm(i - point));

        // ray direction vector points intersection point
        const cv::Vec3f d(ray_direction.at<cv::Vec3f>(y, x));
        const double t((i - ray_origin).dot(d) / d.dot(d));
        EXPECT_TRUE(t >= 0.);
        EXPECT_NEAR(cv::norm(i, ray_origin + t * d), 0., 0.001 * cv::norm(t * d));
      } else {
        const cv::Vec3f d(ray_direction.at<cv::Vec3f>(y, x));
        if (normal.dot(ray_origin - point) >= 0. /* if ray origin is above plane */) {
          // ray direction should point upward
          EXPECT_GE(normal.dot(d), 0.);
        } else /* if ray origin is below plane */ {
          // ray direction should point downward
          EXPECT_LE(normal.dot(d), 0.);
        }
      }
    }
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_plane_surface_model");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}