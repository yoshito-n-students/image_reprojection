#include <image_reprojection/transform_interface.hpp>
#include <image_reprojection_plugins/no_transform.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::NoTransform,
                       image_reprojection::TransformInterface);
