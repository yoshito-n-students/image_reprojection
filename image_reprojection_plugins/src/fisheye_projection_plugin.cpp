#include <image_reprojection/projection_interface.hpp>
#include <image_reprojection_plugins/fisheye_projection.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::FisheyeProjection,
                       image_reprojection::ProjectionInterface);