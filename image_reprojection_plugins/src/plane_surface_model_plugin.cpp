#include <image_reprojection/surface_model.hpp>
#include <image_reprojection_plugins/plane_surface_model.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::PlaneSurfaceModel,
                       image_reprojection::SurfaceModel);
