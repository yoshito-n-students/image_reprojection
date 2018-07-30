#include <gtest/gtest.h>

#include <image_reprojection/camera_model.hpp>
#include <image_reprojection_plugins/fisheye_camera_model.hpp>
#include <image_reprojection_plugins/pinhole_camera_model.hpp>

#include <opencv2/core/core.hpp>

// the global random number generator
cv::RNG g_rng(std::time(NULL));

testCameraModel() {}

TEST(PinholeCameraModel, 3dToPixelTo3dRay) {}

TEST(PinholeCameraModel, pixelTo3dRayToPixel) {}

TEST(FisheyeCameraModel, 3dToPixelTo3dRay) {}

TEST(FisheyeCameraModel, pixelTo3dRayToPixel) {}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}