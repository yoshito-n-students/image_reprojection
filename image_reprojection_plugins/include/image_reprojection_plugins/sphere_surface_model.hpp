#ifndef IMAGE_REPROJECTION_PLUGINS_SPHERE_SURFACE_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_SPHERE_SURFACE_MODEL_HPP

#include <cmath>
#include <string>

#include <image_reprojection/surface_model.hpp>
#include <image_reprojection_plugins/SphereStamped.h>
#include <topic_tools/shape_shifter.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class SphereSurfaceModel : public image_reprojection::SurfaceModel {
public:
  SphereSurfaceModel() {}

  virtual ~SphereSurfaceModel() {}

  virtual void update(const topic_tools::ShapeShifter &surface) {
    const SphereStampedConstPtr sphere(surface.instantiate< SphereStamped >());
    CV_Assert(sphere);
    update(*sphere);
  }

  void update(const SphereStamped &sphere) {
    CV_Assert(sphere.radius > 0.);

    boost::unique_lock< boost::shared_mutex > write_lock(mutex_);

    frame_id_ = sphere.header.frame_id;
    center_ = cv::Vec3f(sphere.center.x, sphere.center.y, sphere.center.z);
    radius_ = sphere.radius;
  }

  virtual std::string getFrameId() const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);
    return frame_id_;
  }

private:
  virtual void onInit() {}

  virtual void onIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                              cv::Mat &dst, cv::Mat &mask) const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);
    multiraySphereIntersection(src_origin, src_direction, dst, mask);
  }

  void multiraySphereIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                                  cv::Mat &dst, cv::Mat &mask) const {
    dst.create(src_direction.size(), CV_32FC3);
    for (int x = 0; x < src_direction.size().width; ++x) {
      for (int y = 0; y < src_direction.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Vec3f &sd(src_direction.at< cv::Vec3f >(y, x));
        cv::Vec3f &d(dst.at< cv::Vec3f >(y, x));
        m = (m != 0 && raySphereIntersection(src_origin, sd, d)) ? 1 : 0;
      }
    }
  }

  bool raySphereIntersection(const cv::Vec3f &src_origin, const cv::Vec3f &src_direction,
                             cv::Vec3f &dst) const {
    // intersection point (x) can be described as
    //   x = p + t * d  (t >= 0)
    //   |x - c| = r
    // where p: ray origin, d: ray direction, c: center of sphere, r: radius of sphere
    //   |d|^2 * t^2 + 2 * dot(d, p - c) * t + |p - c|^2 - r^2 = 0

    // position of ray origin with respect to center of sphere
    const cv::Vec3f o(src_origin - center_);

    // coefficients
    const double a(src_direction.dot(src_direction));
    const double b(src_direction.dot(o));
    const double c(o.dot(o) - radius_ * radius_);
    const double D(b * b - a * c); // discriminant
    if (/* no intersection */ D < 0.) {
      return false;
    }

    // intersection point
    const double sD(std::sqrt(D));
    const double t0((-b + sD) / a), t1((-b - sD) / a);
    if (t0 >= 0. && t1 >= 0.) {
      dst = src_origin + std::min(t0, t1) * src_direction;
      return true;
    } else if (t0 >= 0. && t1 < 0.) {
      dst = src_origin + t0 * src_direction;
      return true;
    } else if (t0 < 0. && t1 >= 0.) {
      dst = src_origin + t1 * src_direction;
      return true;
    }

    return false;
  }

private:
  mutable boost::shared_mutex mutex_;
  std::string frame_id_;
  cv::Vec3f center_;
  double radius_;
};

} // namespace image_reprojection_plugins

#endif