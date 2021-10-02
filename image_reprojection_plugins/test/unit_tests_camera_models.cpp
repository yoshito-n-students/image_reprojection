#include <gtest/gtest.h>

#include <image_reprojection/camera_model.hpp>
#include <image_reprojection_plugins/fisheye_camera_model.hpp>
#include <image_reprojection_plugins/pinhole_camera_model.hpp>

#include <opencv2/core/core.hpp>

#include "random.hpp"

namespace ir = image_reprojection;
namespace irp = image_reprojection_plugins;

//
// test backends
//

void test3dToPixelTo3dRay(const ir::CameraModel &model, const cv::Mat &points1) {
  // project 3d points to pixels
  cv::Mat pixels, mask1 = cv::Mat::ones(points1.size(), CV_8UC1);
  model.project3dToPixel(points1, pixels, mask1);

  // project pixels to 3d rays
  cv::Mat points2, mask2 = mask1.clone();
  model.projectPixelTo3dRay(pixels, points2, mask2);

  // compare results
  std::size_t n_point_comparizon = 0;
  for (int x = 0; x < points1.size().width; ++x) {
    for (int y = 0; y < points1.size().height; ++y) {
      // expectation:
      //   if point-to-pixel projection is valid, the opposite projection is also valid.
      //   so mask1 for former projection and mask2 for latter should match.
      const unsigned char m1 = mask1.at<unsigned char>(y, x);
      const unsigned char m2 = mask2.at<unsigned char>(y, x);
      EXPECT_TRUE((m1 == 0 && m2 == 0) || (m1 != 0 && m2 != 0));
      // expectation:
      //   direction of source point and final ray should match.
      if (m1 != 0) {
        const cv::Point3f pn1 = cv::normalize(points1.at<cv::Vec3f>(y, x));
        const cv::Point3f pn2 = cv::normalize(points2.at<cv::Vec3f>(y, x));
        EXPECT_NEAR(pn1.x, pn2.x, 0.001);
        EXPECT_NEAR(pn1.y, pn2.y, 0.001);
        EXPECT_NEAR(pn1.z, pn2.z, 0.001);
        ++n_point_comparizon;
      }
    }
  }
  ASSERT_TRUE(n_point_comparizon > 0); // make sure point comparison was performed
}

void testPixelTo3dRayToPixel(const ir::CameraModel &model, const cv::Mat &pixels1) {
  // project pixels to 3d rays
  cv::Mat rays, mask1 = cv::Mat::ones(pixels1.size(), CV_8UC1);
  model.projectPixelTo3dRay(pixels1, rays, mask1);

  // project 3d rays to pixels
  cv::Mat pixels2, mask2 = mask1.clone();
  model.project3dToPixel(rays, pixels2, mask2);

  // compare results
  std::size_t n_pixel_comparizon(0);
  for (int x = 0; x < pixels1.size().width; ++x) {
    for (int y = 0; y < pixels1.size().height; ++y) {
      // expectation:
      //   if pixel-to-ray projection is valid, the opposite projection is also valid.
      //   so mask1 for former projection and mask2 for latter should match.
      const unsigned char m1 = mask1.at<unsigned char>(y, x);
      const unsigned char m2 = mask2.at<unsigned char>(y, x);
      EXPECT_TRUE((m1 == 0 && m2 == 0) || (m1 != 0 && m2 != 0));
      // expectation:
      //   source and final pixels should match.
      if (m1 != 0) {
        const cv::Point2f &p1 = pixels1.at<cv::Point2f>(y, x);
        const cv::Point2f &p2 = pixels2.at<cv::Point2f>(y, x);
        EXPECT_NEAR(p1.x, p2.x, 0.001);
        EXPECT_NEAR(p1.y, p2.y, 0.001);
        ++n_pixel_comparizon;
      }
    }
  }
  ASSERT_TRUE(n_pixel_comparizon > 0); // make sure pixel comparison was performed
}

//
// test frontends
//

TEST(PinholeCameraModel, 3dToPixelTo3dRay) {
  // camera info
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = "plumb_bob";
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.K = {100., 0.,   320., // fx,  0, cx
                   0.,   120., 240., //  0, fy, cy
                   0.,   0.,   1.};

  // init camera model
  irp::PinholeCameraModel model;
  model.init("pinhole", ros::M_string(), ros::V_string());
  model.fromCameraInfo(camera_info);

  // perform test with random points
  test3dToPixelTo3dRay(model, randomPoints(cv::Size(100, 100), -1., 1.));
}

TEST(PinholeCameraModel, pixelTo3dRayToPixel) {
  // camera info
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = "plumb_bob";
  camera_info.width = 1980;
  camera_info.height = 1080;
  camera_info.K = {1500., 0.,    990., // fx,  0, cx
                   0.,    1600., 540., //  0, fy, cy
                   0.,    0.,    1.};

  // init camera model
  irp::PinholeCameraModel model;
  model.init("pinhole", ros::M_string(), ros::V_string());
  model.fromCameraInfo(camera_info);

  // perform test with random pixels
  const cv::Size num_pixels(100, 100);
  const cv::Rect region(cv::Point(-500, -500), cv::Size(2500, 2000));
  testPixelTo3dRayToPixel(model, randomPixels(num_pixels, region));
}

TEST(FisheyeCameraModel, 3dToPixelTo3dRay) {
  // camera info
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = "fisheye";
  camera_info.width = 1980;
  camera_info.height = 1080;
  camera_info.K = {300., 0.,   495., // fx,  0, cx
                   0.,   300., 540., //  0, fy, cy
                   0.,   0.,   1.};

  // init camera model
  irp::FisheyeCameraModel model;
  model.init("fisheye", ros::M_string(), ros::V_string());
  model.fromCameraInfo(camera_info);

  // perform test with random points
  test3dToPixelTo3dRay(model, randomPoints(cv::Size(100, 100), -1., 1.));
}

TEST(FisheyeCameraModel, pixelTo3dRayToPixel) {
  // camera info
  sensor_msgs::CameraInfo camera_info;
  camera_info.distortion_model = "fisheye";
  camera_info.width = 1980;
  camera_info.height = 1080;
  camera_info.K = {300., 0.,   1485., // fx,  0, cx
                   0.,   300., 540.,  //  0, fy, cy
                   0.,   0.,   1.};

  // init camera model
  irp::FisheyeCameraModel model;
  model.init("fisheye", ros::M_string(), ros::V_string());
  model.fromCameraInfo(camera_info);

  // perform test with random pixels
  const cv::Size num_pixels(100, 100);
  const cv::Rect region(cv::Point(-500, -500), cv::Size(2500, 2000));
  testPixelTo3dRayToPixel(model, randomPixels(num_pixels, region));
}