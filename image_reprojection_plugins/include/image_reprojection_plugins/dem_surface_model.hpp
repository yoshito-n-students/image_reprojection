#ifndef IMAGE_REPROJECTION_PLUGINS_DEM_SURFACE_MODEL_HPP
#define IMAGE_REPROJECTION_PLUGINS_DEM_SURFACE_MODEL_HPP

#include <string>

#include <image_reprojection/surface_model.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
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
    boost::unique_lock< boost::shared_mutex > write_lock(mutex_);
    frame_id_ = dem.header.frame_id;
  }

  virtual std::string getFrameId() const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);
    return frame_id_;
  }

private:
  virtual void onInit() {
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    CV_Assert(pnh.getParam("min_data", min_data_));
    CV_Assert(pnh.getParam("max_data", max_data_));
  }

  virtual void onIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                              cv::Mat &dst, cv::Mat &mask) const {
    boost::shared_lock< boost::shared_mutex > read_lock(mutex_);
  }

private:
  mutable boost::shared_mutex mutex_;
  double min_data_, max_data_;
  std::string frame_id_;
};

} // namespace image_reprojection_plugins

#endif