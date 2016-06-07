#include <image_reprojection/transform_interface.hpp>
#include <image_reprojection_plugins/rotation_transform.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::RotationTransform,
                       image_reprojection::TransformInterface);
