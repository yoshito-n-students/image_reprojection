#ifndef IMAGE_REPROJECTION_PLUGINS_MESH_SURFACE_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_MESH_SURFACE_MODEL_HPP

#include <string>
#include <vector>

#include <image_reprojection/surface_model.hpp>
#include <image_reprojection_plugins/MeshStamped.h>
#include <topic_tools/shape_shifter.h>

#include <boost/foreach.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class MeshSurfaceModel : public image_reprojection::SurfaceModel {
public:
  MeshSurfaceModel() {}

  virtual ~MeshSurfaceModel() {}

private:
  virtual void onInit() {}

  virtual void update(const topic_tools::ShapeShifter &surface) {
    const MeshStampedConstPtr mesh(surface.instantiate< MeshStamped >());
    CV_Assert(mesh);

    frame_id_ = mesh->header.frame_id;

    vertices_.clear();
    BOOST_FOREACH (const geometry_msgs::Point &v, mesh->mesh.vertices) {
      vertices_.push_back(cv::Vec3f(v.x, v.y, v.z));
    }

    triangles_.clear();
    BOOST_FOREACH (const shape_msgs::MeshTriangle &t, mesh->mesh.triangles) {
      triangles_.push_back(
          cv::Vec3i(t.vertex_indices[0], t.vertex_indices[1], t.vertex_indices[2]));
    }
  }

  virtual std::string getFrameId() const { return frame_id_; }

  virtual void onIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                              cv::Mat &dst, cv::Mat &mask) const {
    multirayMeshIntersection(src_origin, src_direction, dst, mask);
  }

  void multirayMeshIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                                cv::Mat &dst, cv::Mat &mask) const {
    dst.create(src_direction.size(), CV_32FC3);
    for (int x = 0; x < src_direction.size().width; ++x) {
      for (int y = 0; y < src_direction.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Vec3f &sd(src_direction.at< cv::Vec3f >(y, x));
        cv::Vec3f &d(dst.at< cv::Vec3f >(y, x));
        m = (m != 0 && rayMeshIntersection(src_origin, sd, d)) ? 1 : 0;
      }
    }
  }

  bool rayMeshIntersection(const cv::Vec3f &src_origin, const cv::Vec3f &src_direction,
                           cv::Vec3f &dst) const {
    bool intersects(false);
    double min_distance;
    BOOST_FOREACH (const cv::Vec3i &triangle, triangles_) {
      cv::Vec3f this_intersection;
      if (rayTriangleIntersection(src_origin, src_direction, triangle, this_intersection)) {
        const double this_distance(cv::norm(src_origin, this_intersection));
        if (!intersects || (intersects && this_distance < min_distance)) {
          intersects = true;
          min_distance = this_distance;
          dst = this_intersection;
        }
      }
    }

    return intersects;
  }

  bool rayTriangleIntersection(const cv::Vec3f &src_origin, const cv::Vec3f &src_direction,
                               const cv::Vec3i &triangle, cv::Vec3f &dst) const {
    // primitive vectors
    const cv::Vec3f r(src_origin - vertices_[triangle[0]]);              // position of ray origin
    const cv::Vec3f e1(vertices_[triangle[1]] - vertices_[triangle[0]]); // 1st edge of triangle
    const cv::Vec3f e2(vertices_[triangle[2]] - vertices_[triangle[0]]); // 2nd edge of triangle
    const cv::Vec3f n(e1.cross(e2));                                     // normal of triangle

    // intersection point (p) can be described as
    //    p = r + t * d = b1 * e1 + b2 * e2
    //    (t >= 0, b1 >= 0, b2 >= 0, b1 + b2 <= 1)
    // where d is ray direction

    // calculate dot(d, n) for convenience
    const double ddn(src_direction.dot(n));
    if (ddn == 0.) { // means ray and triangle are parallel
      return false;
    }
    const double ddn_sign(ddn > 0. ? 1. : -1.);
    const double ddn_norm(ddn > 0. ? ddn : -ddn);

    // t = dot(r, n) / dot(d, n)
    const double srdn(ddn_sign * r.dot(n));
    if (srdn < 0.) { // means t < 0
      return false;
    }

    // b1 = dot(d, cross(r, e2)) / dot(d, n)
    const double sddrce2(ddn_sign * src_direction.dot(r.cross(e2)));
    if (sddrce2 < 0. || sddrce2 > ddn_norm) { // means b1 < 0 or b1 > 1
      return false;
    }

    // b2 = dot(d, cross(e1, r)) / dot(d, n)
    const double sdde1cr(ddn_sign * src_direction.dot(e1.cross(r)));
    if (sdde1cr < 0. || sdde1cr > ddn_norm) { // means b2 < 0 or b2 > 1
      return false;
    }

    if (sddrce2 + sdde1cr > ddn_norm) { // means b1 + b2 > 1
      return false;
    }

    dst = r + srdn * src_direction / ddn_norm;
    return true;
  }

private:
  std::string frame_id_;
  std::vector< cv::Vec3f > vertices_;
  std::vector< cv::Vec3i > triangles_;
};

} // namespace image_reprojection_plugins

#endif