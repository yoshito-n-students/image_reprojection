#ifndef IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP
#define IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <image_reprojection/camera_model.hpp>
#include <image_reprojection/surface_model.hpp>
#include <image_reprojection/transform.hpp>
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
    surface_subscriber_.shutdown();
    dst_camera_info_server_.shutdown();
    map_timer_.stop();

    // destroy all plugins before destroying loaders
    src_camera_model_.reset();
    surface_model_.reset();
    dst_camera_model_.reset();
  }

private:
  //
  // initialization
  //

  virtual void onInit() {
    // get node handles
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    //
    // src camera (TODO: multiple src cameras)
    //

    // load src camera model instance
    {
      std::string type;
      CV_Assert(pnh.getParam("src_camera0/model", type));
      src_camera_model_ = camera_model_loader_.createInstance(type);
      src_camera_model_->init(pnh.resolveName("src_camera0") + "(" + type + ")", ros::M_string(),
                              getMyArgv(), &getSTCallbackQueue(), &getMTCallbackQueue());
    }

    // subscribe src camera
    {
      const image_transport::TransportHints default_hints;
      src_camera_subscriber_ =
          it.subscribeCamera("src_camera0", 1, &ImageReprojection::onSrcCameraRecieved, this,
                             image_transport::TransportHints(default_hints.getTransport(),
                                                             default_hints.getRosHints(), pnh));
    }

    //
    // surface
    //

    // load surface model instance
    {
      std::string type;
      CV_Assert(pnh.getParam("surface/model", type));
      surface_model_ = surface_model_loader_.createInstance(type);
      surface_model_->init(pnh.resolveName("surface") + "(" + type + ")", ros::M_string(),
                           getMyArgv(), &getSTCallbackQueue(), &getMTCallbackQueue());
    }

    // setup the surface subscriber
    surface_subscriber_ = nh.subscribe("surface", 1, &ImageReprojection::onSurfaceRecieved, this);

    //
    // dst camera and map update
    //

    // load parameters
    map_binning_x_ = pnh.param("map_update/binning_x", 5);
    map_binning_y_ = pnh.param("map_update/binning_y", 5);

    // setup the coordinate frame transformer
    tf_listener_.reset(new tf::TransformListener(nh));

    // load dst camera instance
    {
      std::string type;
      CV_Assert(pnh.getParam("dst_camera/model", type));
      dst_camera_model_ = camera_model_loader_.createInstance(type);
      dst_camera_model_->init(pnh.resolveName("dst_camera") + "(" + type + ")", ros::M_string(),
                              getMyArgv(), &getSTCallbackQueue(), &getMTCallbackQueue());
    }

    // init dst camera model
    {
      std::string info_file, name;
      CV_Assert(pnh.getParam("dst_camera/info_file", info_file));
      CV_Assert(pnh.getParam("dst_camera/name", name));
      sensor_msgs::CameraInfo info;
      CV_Assert(camera_calibration_parsers::readCalibration(info_file, name, info));
      dst_camera_model_->fromCameraInfo(info);
    }

    // setup the destination camera info updater
    dst_camera_info_server_ =
        nh.advertiseService("set_camera_info", &ImageReprojection::onDstCameraInfoSet, this);

    // setup the destination image publisher
    dst_camera_publisher_ = it.advertiseCamera("dst_camera", 1, true);

    // start publish timer
    {
      const ros::Duration period(ros::Rate(pnh.param("dst_camera/fps", 16.)).expectedCycleTime());
      if (pnh.param("map_update/background", false)) {
        map_timer_ = nh.createTimer(ros::Rate(pnh.param("map_update/frequency", 8.)),
                                    &ImageReprojection::onMapUpdateEvent, this);
        dst_camera_timer_ = nh.createTimer(
            period, boost::bind(&ImageReprojection::onDstCameraEvent, this, _1, false));
      } else {
        dst_camera_timer_ = nh.createTimer(
            period, boost::bind(&ImageReprojection::onDstCameraEvent, this, _1, true));
      }
    }
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

  //
  // passive event handlers
  //

  void onSrcCameraRecieved(const sensor_msgs::ImageConstPtr &src_image,
                           const sensor_msgs::CameraInfoConstPtr &src_camera_info) {
    try {
      // update source camera model
      src_camera_model_->fromCameraInfo(*src_camera_info);

      // convert the ROS source image to an opencv image
      src_image_ = cv_bridge::toCvShare(*src_image, src_image);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onSrcRecieved: " << ex.what());
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

  //
  // scheduled event handlers
  //

  void onDstCameraEvent(const ros::TimerEvent &event, const bool do_update_map) {
    try {
      // do nothing if there is no node that subscribes this node
      if (dst_camera_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // get destination camera info
      const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
      CV_Assert(dst_camera_info);

      // prepare the destination image
      cv_bridge::CvImage dst_image;
      CV_Assert(src_image_);
      dst_image.header.stamp = src_image_->header.stamp;
      dst_image.header.frame_id = dst_camera_info->header.frame_id;
      dst_image.encoding = src_image_->encoding;

      if (do_update_map) {
        // update mapping between the source and destination images
        // (this elapses 90+% of excecution time of this function
        //  and can be done without receiving the source image.
        //  this is why the fast mode is requred for some application.)
        updateMap();
      }

      // fill the destination image by remapping the source image
      remap(src_image_->image, dst_image.image);

      // publish the destination image
      dst_camera_publisher_.publish(dst_image.toImageMsg(), dst_camera_info);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onDstCameraEvent: " << ex.what());
    }
  }

  void onMapUpdateEvent(const ros::TimerEvent &event) {
    try {
      updateMap();
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onMapUpdateEvent: " << ex.what());
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
    cv::Vec3f ray_origin;
    {
      tf::StampedTransform dst2surface;
      tf_listener_->lookupTransform(dst_camera_info->header.frame_id, surface_model_->getFrameId(),
                                    ros::Time(0), dst2surface);
      ray_origin = transform(cv::Vec3f(0., 0., 0.), dst2surface);
      ray_directions = transform(ray_directions, dst2surface.getBasis(), binned_mask);
    }

    // calculate mapping from ray to intersection point on surface
    cv::Mat intersections;
    surface_model_->intersection(ray_origin, ray_directions, intersections, binned_mask);

    // TODO: check intersection points are visible from src camera origin using the surface model

    // transform intersection points into camera frame
    {
      const sensor_msgs::CameraInfoConstPtr src_camera_info(src_camera_model_->toCameraInfo());
      CV_Assert(src_camera_info);
      tf::StampedTransform surface2src;
      tf_listener_->lookupTransform(surface_model_->getFrameId(), src_camera_info->header.frame_id,
                                    ros::Time(0), surface2src);
      intersections = transform(intersections, surface2src, binned_mask);
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

private:
  // model loaders
  pluginlib::ClassLoader< CameraModel > camera_model_loader_;
  pluginlib::ClassLoader< SurfaceModel > surface_model_loader_;

  // src camera
  image_transport::CameraSubscriber src_camera_subscriber_;
  CameraModelPtr src_camera_model_;
  cv_bridge::CvImageConstPtr src_image_;

  // surface
  ros::Subscriber surface_subscriber_;
  SurfaceModelPtr surface_model_;

  // dst camera
  ros::ServiceServer dst_camera_info_server_;
  CameraModelPtr dst_camera_model_;
  image_transport::CameraPublisher dst_camera_publisher_;
  ros::Timer dst_camera_timer_;

  // mapping between src and dst image pixels
  ros::Timer map_timer_;
  boost::scoped_ptr< tf::TransformListener > tf_listener_;
  int map_binning_x_;
  int map_binning_y_;
  boost::mutex mutex_;
  cv::Mat map_;
  cv::Mat mask_;
};

} // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
