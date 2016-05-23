#include <image_reprojection/projection_interface.hpp>
#include <image_reprojection_plugins/standard_projection.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::StandardProjection,
                       image_reprojection::ProjectionInterface);
