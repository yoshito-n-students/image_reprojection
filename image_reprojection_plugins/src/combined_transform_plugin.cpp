#include <image_reprojection/transform_interface.hpp>
#include <image_reprojection_plugins/combined_transform.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::CombinedTransform,
                       image_reprojection::TransformInterface);
