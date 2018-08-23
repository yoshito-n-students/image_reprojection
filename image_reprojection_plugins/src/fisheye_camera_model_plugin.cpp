#include <image_reprojection/camera_model.hpp>
#include <image_reprojection_plugins/fisheye_camera_model.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::FisheyeCameraModel,
                       image_reprojection::CameraModel);
