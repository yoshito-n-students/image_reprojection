#include <image_reprojection/transform_plugin.hpp>
#include <pluginlib/class_list_macros.h>
#include <image_reprojection_plugins/no_transform_plugin.hpp>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::NoTransformPlugin,
		       image_reprojection::TransformPlugin);
