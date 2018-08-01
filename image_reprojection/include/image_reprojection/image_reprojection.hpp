#ifndef IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP
#define IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <image_reprojection/camera_model.hpp>
#include <image_reprojection/surface_model.hpp>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/timer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <tf/transform_listener.h>
#include <topic_tools/shape_shifter.h>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection {

class ImageReprojection : public nodelet::Nodelet {
public:
  ImageReprojection()
      : camera_model_loader_("image_reprojection", "image_reprojection::CameraModel"),
        surface_model_loader_("image_reprojection", "image_reprojection::SurfaceModel") {}

  virtual ~ImageReprojection() {
    // stop using plugins
    src_camera_subscriber_.shutdown();
    dst_camera_info_server_.shutdown();
    map_timer_.stop();

    // destroy all plugins before destroying loaders
    src_camera_model_.reset();
    dst_camera_model_.reset();
    surface_model_.reset();
  }

private:
  virtual void onInit() {
    // get node handles
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    // load plugins
    {
      std::string type;
      CV_Assert(pnh.getParam("src_camera0/model", type));
      src_camera_model_ = camera_model_loader_.createInstance(type);
    }
    {
      std::string type;
      CV_Assert(pnh.getParam("dst_camera/model", type));
      dst_camera_model_ = camera_model_loader_.createInstance(type);
    }
    {
      std::string type;
      CV_Assert(pnh.getParam("surface/model", type));
      surface_model_ = surface_model_loader_.createInstance(type);
    }

    // init destination camera model
    {
      std::string info_file, name;
      CV_Assert(pnh.getParam("dst_camera/info_file", info_file));
      CV_Assert(pnh.getParam("dst_camera/name", name));
      sensor_msgs::CameraInfo info;
      CV_Assert(camera_calibration_parsers::readCalibration(info_file, name, info));
      dst_camera_model_->fromCameraInfo(info);
    }

    // load parameters
    map_binning_x_ = pnh.param("map_update/binning_x", 5);
    map_binning_y_ = pnh.param("map_update/binning_y", 5);

    // setup the coordinate frame transformer
    tf_listener_.reset(new tf::TransformListener(nh));

    // setup the destination image publisher
    dst_camera_publisher_ = it.advertiseCamera("dst_camera", 1, true);

    // setup the destination camera info updater
    dst_camera_info_server_ =
        nh.advertiseService("set_camera_info", &ImageReprojection::onDstCameraInfoSet, this);

    // setup the surface subscriber
    surface_subscriber_ = nh.subscribe("surface", 1, &ImageReprojection::onSurfaceRecieved, this);

    // start the source image subscriber
    // (TODO: launch multiple subscribers)
    if (pnh.param("map_update/background", false)) {
      // background map updater
      map_timer_ = nh.createTimer(ros::Rate(pnh.param("map_update/frequency", 5.)),
                                  &ImageReprojection::onMapUpdateEvent, this);
      // source image subscriber not updating map
      const image_transport::TransportHints default_hints;
      src_camera_subscriber_ = it.subscribeCamera(
          "src_camera0", 1,
          boost::bind(&ImageReprojection::onSrcCameraRecieved, this, _1, _2, false), ros::VoidPtr(),
          image_transport::TransportHints(default_hints.getTransport(), default_hints.getRosHints(),
                                          pnh));
    } else {
      // source image subscriber updating map
      const image_transport::TransportHints default_hints;
      src_camera_subscriber_ = it.subscribeCamera(
          "src_camera0", 1,
          boost::bind(&ImageReprojection::onSrcCameraRecieved, this, _1, _2, true), ros::VoidPtr(),
          image_transport::TransportHints(default_hints.getTransport(), default_hints.getRosHints(),
                                          pnh));
    }
  }

  void onSurfaceRecieved(const topic_tools::ShapeShifter::ConstPtr &surface) {
    try {
      surface_model_->update(*surface);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onSurfaceRecieved: " << ex.what());
    }
  }

  bool onDstCameraInfoSet(sensor_msgs::SetCameraInfo::Request &request,
                          sensor_msgs::SetCameraInfo::Response &response) {
    try {
      dst_camera_model_->fromCameraInfo(request.camera_info);
      response.success = true;
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onDstCameraInfoSet: " << ex.what());
      response.success = false;
      response.status_message = ex.what();
    }
    return response.success;
  }

  void onSrcCameraRecieved(const sensor_msgs::ImageConstPtr &ros_src,
                           const sensor_msgs::CameraInfoConstPtr &src_camera_info,
                           const bool do_update_map) {
    try {
      // update source camera model
      src_camera_model_->fromCameraInfo(*src_camera_info);

      // do nothing else if there is no node that subscribes this node
      if (dst_camera_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // convert the ROS source image to an opencv image
      cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

      // get destination camera info
      const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
      CV_Assert(dst_camera_info);

      // prepare the destination image
      cv_bridge::CvImage cv_dst;
      cv_dst.header.seq = cv_src->header.seq;
      cv_dst.header.stamp = cv_src->header.stamp;
      cv_dst.header.frame_id = dst_camera_info->header.frame_id;
      cv_dst.encoding = cv_src->encoding;

      if (do_update_map) {
        // update mapping between the source and destination images
        // (this elapses 90+% of excecution time of this function
        //  and can be done without receiving the source image.
        //  this is why the fast mode is requred for some application.)
        updateMap();
      }

      // fill the destination image by remapping the source image
      remap(cv_src->image, cv_dst.image);

      // publish the destination image
      dst_camera_publisher_.publish(cv_dst.toImageMsg(), dst_camera_info);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onSrcRecieved: " << ex.what());
    }
  }

  void onMapUpdateEvent(const ros::TimerEvent &event) {
    try {
      updateMap();
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onMapUpdateEvent:" << ex.what());
    }
  }

  void updateMap() {
    // get destination camera info
    const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
    CV_Assert(dst_camera_info);

    // figure out destination image size
    const cv::Size dst_image_size(
        (dst_camera_info->roi.width == 0 ? dst_camera_info->width : dst_camera_info->roi.width) /
            (dst_camera_info->binning_x == 0 ? 1 : dst_camera_info->binning_x),
        (dst_camera_info->roi.height == 0 ? dst_camera_info->height : dst_camera_info->roi.height) /
            (dst_camera_info->binning_y == 0 ? 1 : dst_camera_info->binning_y));

    // create initial map and mask
    cv::Mat binned_map, binned_mask;
    {
      cv::Mat full_map(dst_image_size, CV_32FC2);
      for (int x = 0; x < dst_image_size.width; ++x) {
        for (int y = 0; y < dst_image_size.height; ++y) {
          // center coordinate of the pixel
          full_map.at< cv::Point2f >(y, x) = cv::Point2f(x + 0.5, y + 0.5);
        }
      }
      const cv::Size binned_map_size(dst_image_size.width / map_binning_x_,
                                     dst_image_size.height / map_binning_y_);
      cv::resize(full_map, binned_map, binned_map_size);
      binned_mask = cv::Mat::ones(binned_map_size, CV_8UC1);
    }

    // calculate mapping from destination pixels to ray toward surface
    cv::Mat ray_directions;
    dst_camera_model_->projectPixelTo3dRay(binned_map, ray_directions, binned_mask);

    // transform rays into surface coordinate frame
    cv::Vec3f ray_origin(0., 0., 0.);
    {
      tf::StampedTransform dst2surface;
      tf_listener_->lookupTransform(dst_camera_info->header.frame_id, surface_model_->getFrameId(),
                                    ros::Time(0), dst2surface);
      transform(ray_origin, dst2surface);
      transform(ray_directions, dst2surface, binned_mask);
    }

    // calculate mapping from ray to intersection point on surface
    cv::Mat intersections;
    surface_model_->intersection(ray_origin, ray_directions, intersections, binned_mask);

    // transform intersection points into camera frame
    {
      const sensor_msgs::CameraInfoConstPtr src_camera_info(src_camera_model_->toCameraInfo());
      CV_Assert(src_camera_info);
      tf::StampedTransform surface2src;
      tf_listener_->lookupTransform(surface_model_->getFrameId(), src_camera_info->header.frame_id,
                                    ros::Time(0), surface2src);
      transform(intersections, surface2src, binned_mask);
    }

    // calculate mapping from points on surface to source pixels
    src_camera_model_->project3dToPixel(intersections, binned_map, binned_mask);

    // write updated mapping between source and destination images
    {
      boost::lock_guard< boost::mutex > lock(mutex_);
      cv::resize(binned_map, map_, dst_image_size);
      cv::resize(binned_mask, mask_, dst_image_size);
    }
  }

  void remap(const cv::Mat &src, cv::Mat &dst) {
    boost::lock_guard< boost::mutex > lock(mutex_);

    // remap the source image to a temp image
    cv::Mat tmp;
    cv::remap(src, tmp, map_, cv::noArray(), cv::INTER_LINEAR);

    // copy unmasked pixels of the temp image to the destination image
    dst = cv::Mat::zeros(map_.size(), src.type());
    tmp.copyTo(dst, mask_);
  }

  static cv::Size sizeParam(ros::NodeHandle &pnh, const std::string &key,
                            const cv::Size &default_val) {
    std::vector< int > val;
    if (pnh.getParam(key, val)) {
      if (val.size() == 2) {
        return cv::Size(val[0], val[1]);
      }
    }
    return default_val;
  }

  static void transform(cv::Vec3f &cv_vec, const tf::Transform &tf_transform) {
    const tf::Vector3 tf_vec(tf_transform(tf::Vector3(cv_vec[0], cv_vec[1], cv_vec[2])));
    cv_vec[0] = tf_vec[0];
    cv_vec[1] = tf_vec[1];
    cv_vec[2] = tf_vec[2];
  }

  static void transform(cv::Mat &cv_mat, const tf::Transform &tf_transform, const cv::Mat &mask) {
    CV_Assert(cv_mat.type() == CV_32FC3);
    CV_Assert(mask.type() == CV_8UC1);
    CV_Assert(cv_mat.size() == mask.size());
    for (int x = 0; x < cv_mat.size().width; ++x) {
      for (int y = 0; y < cv_mat.size().height; ++y) {
        if (mask.at< unsigned char >(y, x) != 0) {
          transform(cv_mat.at< cv::Vec3f >(y, x), tf_transform);
        }
      }
    }
  }

private:
  // model loaders
  pluginlib::ClassLoader< CameraModel > camera_model_loader_;
  pluginlib::ClassLoader< SurfaceModel > surface_model_loader_;

  // models
  CameraModelPtr src_camera_model_;
  CameraModelPtr dst_camera_model_;
  SurfaceModelPtr surface_model_;

  // subscriber for source image and publisher for destination image
  image_transport::CameraSubscriber src_camera_subscriber_;
  ros::Subscriber surface_subscriber_;
  image_transport::CameraPublisher dst_camera_publisher_;
  ros::ServiceServer dst_camera_info_server_;
  ros::Timer map_timer_;

  // transformer
  boost::scoped_ptr< tf::TransformListener > tf_listener_;

  // parameters
  int map_binning_x_;
  int map_binning_y_;

  // mapping between source and destination images
  boost::mutex mutex_;
  cv::Mat map_;
  cv::Mat mask_;
};

} // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
