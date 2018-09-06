#ifndef IMAGE_REPROJECTION_PLUGINS_DEM_SURFACE_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_DEM_SURFACE_MODEL_HPP

#include <limits>
#include <string>

#include <image_reprojection/surface_model.hpp>
#include <image_reprojection/transform.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>
#include <topic_tools/shape_shifter.h>

#include <opencv2/core/core.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>

namespace image_reprojection_plugins {

class DEMSurfaceModel : public image_reprojection::SurfaceModel {
public:
  DEMSurfaceModel() {}

  virtual ~DEMSurfaceModel() {}

  virtual void update(const topic_tools::ShapeShifter &surface) {
    const nav_msgs::OccupancyGridConstPtr dem(surface.instantiate< nav_msgs::OccupancyGrid >());
    CV_Assert(dem);
    update(*dem);
  }

  void update(const nav_msgs::OccupancyGrid &dem) {
    CV_Assert(dem.info.width > 0);
    CV_Assert(dem.info.height > 0);
    CV_Assert(dem.data.size() == dem.info.width * dem.info.height);

    boost::unique_lock< boost::shared_mutex > write_lock(mutex_);

    // frame id of arguments of intersection()
    tf_frame_id_ = dem.header.frame_id;

    // frame conversion between DEM and tf frames
    tf::poseMsgToTF(dem.info.origin, dem2tf_);
    tf2dem_ = dem2tf_.inverse();

    // resolution of DEM (xy)
    resolution_ = dem.info.resolution;

    // elevation data (z)
    data_.create(dem.info.width, dem.info.height, CV_64FC1);
    const double data_scale((max_data_ - min_data_) / 100.);
    for (int xid = 0; xid < dem.info.width; ++xid) {
      for (int yid = 0; yid < dem.info.height; ++yid) {
        const int did(xid + yid * dem.info.width);
        data_.at< double >(yid, xid) = min_data_ + data_scale * dem.data[did];
      }
    }
  }

  virtual std::string getFrameId() const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);
    return tf_frame_id_;
  }

private:
  virtual void onInit() {
    boost::unique_lock< boost::shared_mutex > write_lock(mutex_);

    ros::NodeHandle &pnh(getPrivateNodeHandle());
    min_data_ = pnh.param("min_data", 0.);
    max_data_ = pnh.param("max_data", 1.);
  }

  virtual void onIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                              cv::Mat &dst, cv::Mat &mask) const {
    namespace ir = image_reprojection;

    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);

    multirayDEMIntersection(ir::transform(src_origin, tf2dem_),
                            ir::transform(src_direction, tf2dem_.getBasis(), mask), dst, mask);
    dst = ir::transform(dst, dem2tf_, mask);
  }

  void multirayDEMIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                               cv::Mat &dst, cv::Mat &mask) const {
    dst.create(src_direction.size(), CV_32FC3);
    for (int x = 0; x < src_direction.size().width; ++x) {
      for (int y = 0; y < src_direction.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Vec3f &sd(src_direction.at< cv::Vec3f >(y, x));
        cv::Vec3f &d(dst.at< cv::Vec3f >(y, x));
        m = (m != 0 && rayDEMIntersection(src_origin, sd, d)) ? 1 : 0;
      }
    }
  }

  bool rayDEMIntersection(const cv::Vec3f &src_origin, const cv::Vec3f &src_direction,
                          cv::Vec3f &dst) const {
    bool intersects(false);
    double min_distance;
    for (int xid = 0; xid < data_.size().width; ++xid) {
      for (int yid = 0; yid < data_.size().height; ++yid) {
        cv::Vec3f this_intersection;
        if (rayBoxIntersection(src_origin, src_direction, xid, yid, this_intersection)) {
          const double this_distance(cv::norm(src_origin, this_intersection));
          if (!intersects || (intersects && this_distance < min_distance)) {
            intersects = true;
            min_distance = this_distance;
            dst = this_intersection;
          }
        }
      }
    }

    return intersects;
  }

  bool rayBoxIntersection(const cv::Vec3f &src_origin, const cv::Vec3f &src_direction,
                          const int xid, const int yid, cv::Vec3f &dst) const {
    // box specs
    const cv::Vec3d box_origin(xid * resolution_, yid * resolution_, 0.);
    const cv::Vec3d box_length(resolution_, resolution_, data_.at< double >(yid, xid));

    // intersection point (p) can bescribed as
    //   p = r + t * d
    //   (r: ray origin, d: ray direction, t >= 0)

    // calculate possible range of t in each dimension
    cv::Vec3d t_1d_min, t_1d_max;
    for (int i = 0; i < 3; ++i) {
      if (!raySegmentIntersection(src_origin[i], src_direction[i], box_origin[i],
                                  box_origin[i] + box_length[i], t_1d_min[i], t_1d_max[i])) {
        return false;
      }
    }

    // check if overlap of range of t exists
    double t_min, t_max;
    cv::minMaxIdx(t_1d_min, NULL, &t_min);
    cv::minMaxIdx(t_1d_max, &t_max);
    if (/* no overlap */ t_min > t_max) {
      return false;
    }

    // return the nearest intersection point in ray direction
    dst = src_origin + t_min * src_direction;
    return true;
  }

  bool raySegmentIntersection(const double src_origin, const double src_direction,
                              const double seg_min, const double seg_max, double &t_min,
                              double &t_max) const {
    // o: ray origin
    // ->, <-: ray direction
    // m, M: segment (m <= M)

    if (src_direction == 0.) {
      // o --- m --- M  OR  m --- M --- o
      if (src_origin < seg_min || src_origin > seg_max) {
        return false;
      }
      // m --- o --- M
      t_min = 0.;
      t_max = std::numeric_limits< double >::max();
      return true;
    } else if (src_direction > 0.) {
      // m --- M ---- o->
      if (src_origin > seg_max) {
        return false;
      }
      // m --- o-> --- M
      if (src_origin > seg_min) {
        t_min = 0.;
        t_max = (seg_max - src_origin) / src_direction;
        return true;
      }
      // o-> --- m ---- M
      t_min = (seg_min - src_origin) / src_direction;
      t_max = (seg_max - src_origin) / src_direction;
      return true;
    } else /* src_direction < 0. */ {
      // <-o --- m --- M
      if (src_origin < seg_min) {
        return false;
      }
      // m --- <-o --- M
      if (src_origin < seg_max) {
        t_min = 0.;
        t_max = (seg_min - src_origin) / src_direction;
        return true;
      }
      // m --- M --- <-o
      t_min = (seg_max - src_origin) / src_direction;
      t_max = (seg_min - src_origin) / src_direction;
      return true;
    }

    static const bool NEVER_REACH_HERE_BUG(false);
    CV_Assert(NEVER_REACH_HERE_BUG);
  }

private:
  mutable boost::shared_mutex mutex_;

  // info from nav_msgs::OccupancyGrid
  std::string tf_frame_id_;
  tf::Transform dem2tf_, tf2dem_;
  double resolution_;
  cv::Mat data_;

  // additional info from rosparam
  double min_data_, max_data_;
};

} // namespace image_reprojection_plugins

#endif