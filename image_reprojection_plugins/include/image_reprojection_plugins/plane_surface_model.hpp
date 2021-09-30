#ifndef IMAGE_REPROJECTION_PLUGINS_PLANE_SURFACE_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_PLANE_SURFACE_MODEL_HPP

#include <string>

#include <image_reprojection/surface_model.hpp>
#include <image_reprojection_plugins/PlaneStamped.h>
#include <topic_tools/shape_shifter.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class PlaneSurfaceModel : public image_reprojection::SurfaceModel {
public:
  PlaneSurfaceModel() {}

  virtual ~PlaneSurfaceModel() {}

  virtual void update(const topic_tools::ShapeShifter &surface) override {
    const PlaneStampedConstPtr plane = surface.instantiate<PlaneStamped>();
    CV_Assert(plane);
    update(*plane);
  }

  void update(const PlaneStamped &plane) {
    CV_Assert(plane.normal.x != 0. || plane.normal.y != 0. || plane.normal.z != 0.);

    boost::unique_lock<boost::shared_mutex> write_lock(mutex_);

    frame_id_ = plane.header.frame_id;
    point_ = cv::Vec3f(plane.point.x, plane.point.y, plane.point.z);
    normal_ = cv::Vec3f(plane.normal.x, plane.normal.y, plane.normal.z);
  }

  virtual std::string getFrameId() const {
    boost::shared_lock<boost::shared_mutex> read_lock(mutex_);
    return frame_id_;
  }

private:
  virtual void onInit() override {}

  virtual void onIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                              cv::Mat &dst, cv::Mat &mask) const override {
    boost::shared_lock<boost::shared_mutex> read_lock(mutex_);
    multirayPlaneIntersection(src_origin, src_direction, dst, mask);
  }

  void multirayPlaneIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                                 cv::Mat &dst, cv::Mat &mask) const {
    dst.create(src_direction.size(), CV_32FC3);
    for (int x = 0; x < src_direction.size().width; ++x) {
      for (int y = 0; y < src_direction.size().height; ++y) {
        unsigned char &m = mask.at<unsigned char>(y, x);
        const cv::Vec3f &sd = src_direction.at<cv::Vec3f>(y, x);
        cv::Vec3f &d = dst.at<cv::Vec3f>(y, x);
        m = (m != 0 && rayPlaneIntersection(src_origin, sd, d)) ? 1 : 0;
      }
    }
  }

  bool rayPlaneIntersection(const cv::Vec3f &src_origin, const cv::Vec3f &src_direction,
                            cv::Vec3f &dst) const {
    // position of ray origin with respect to principle point of plane
    const cv::Vec3f r = src_origin - point_;

    // intersection point (p) can be described as
    //    p = r + t * d
    //    (t >= 0)
    // where d is ray direction

    // calculate dot(d, n) for convenience
    const double ddn = src_direction.dot(normal_);
    if (ddn == 0.) { // means ray and plane are parallel
      return false;
    }
    const double ddn_sign = ddn > 0. ? 1. : -1.;
    const double ddn_norm = ddn > 0. ? ddn : -ddn;

    // t = -dot(r, n) / dot(d, n)
    const double srdn = ddn_sign * r.dot(normal_);
    if (srdn > 0.) { // means t < 0
      return false;
    }

    // intersection point with respect to plane coordinate frame
    dst = src_origin - srdn * src_direction / ddn_norm;
    return true;
  }

private:
  mutable boost::shared_mutex mutex_;
  std::string frame_id_;
  cv::Vec3f point_, normal_;
};

} // namespace image_reprojection_plugins

#endif