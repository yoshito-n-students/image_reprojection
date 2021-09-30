#include <image_reprojection/surface_model.hpp>
#include <image_reprojection_plugins/dem_surface_model.hpp>
#include <image_reprojection_plugins/mesh_surface_model.hpp>
#include <image_reprojection_plugins/plane_surface_model.hpp>
#include <image_reprojection_plugins/sphere_surface_model.hpp>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::DEMSurfaceModel,
                       image_reprojection::SurfaceModel);
PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::MeshSurfaceModel,
                       image_reprojection::SurfaceModel);
PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::PlaneSurfaceModel,
                       image_reprojection::SurfaceModel);
PLUGINLIB_EXPORT_CLASS(image_reprojection_plugins::SphereSurfaceModel,
                       image_reprojection::SurfaceModel);
