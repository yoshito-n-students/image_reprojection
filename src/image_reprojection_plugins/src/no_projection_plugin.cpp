#include <image_reprojection/projection_plugin.hpp>
#include <pluginlib/class_list_macros.h>
#include <image_reprojection_plugins/no_projection_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::NoProjectionPlugin,
                       image_reprojection::ProjectionPlugin);
