#ifndef IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP
#define IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <camera_info_manager/camera_info_manager.h>
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
#include <ros/timer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
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
    timer_.stop();

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
      src_camera_model_->init(pnh.resolveName("src_camera0"), ros::M_string(), getMyArgv());
    }
    {
      std::string type;
      CV_Assert(pnh.getParam("dst_camera/model", type));
      dst_camera_model_ = camera_model_loader_.createInstance(type);
      dst_camera_model_->init(pnh.resolveName("dst_camera"), ros::M_string(), getMyArgv());
      dst_camera_info_manager_.reset(new camera_info_manager::CameraInfoManager(
          nh, pnh.param< std::string >("dst_camera/name", "dst_camera"),
          pnh.param< std::string >("dst_camera/info_url", "")));
    }
    {
      std::string type;
      CV_Assert(pnh.getParam("surface/model", type));
      surface_model_ = surface_model_loader_.createInstance(type);
      surface_model_->init(pnh.resolveName("surface"), ros::M_string(), getMyArgv());
    }

    // TODO: set dst_image/size from dst_camera_info_manager_->getCameraInfo().
    //       also, use map_update/binning_{x,y} instread of map_update/size
    //       because dst_image/size can dynamically change.
    //       then, initial map should be generated at beggining of updateMap()
    // init mapping between source and destination images
    {
      // load sizes of the initial and final maps (must be initial <= final)
      const cv::Size size_dst(sizeParam(pnh, "dst_image/size", cv::Size(500, 500)));
      const cv::Size size_seed(sizeParam(pnh, "map_update/size", cv::Size(250, 250)));
      CV_Assert(size_dst.width >= size_seed.width && size_dst.height >= size_seed.height);

      // init final map whose size is same as the destination image
      map_.create(size_dst, CV_32FC2);
      for (int x = 0; x < map_.size().width; ++x) {
        for (int y = 0; y < map_.size().height; ++y) {
          map_.at< cv::Point2f >(y, x) = cv::Point2f(x + 0.5, y + 0.5);
        }
      }
      mask_ = cv::Mat::ones(map_.size(), CV_8UC1);

      // init the seed map by resizing the final map
      cv::resize(map_, seed_map_, size_seed);
    }

    // setup the coordinate frame transformer
    tf_listener_.reset(new tf::TransformListener(nh));
    CV_Assert(tf_listener_);

    // setup the destination image publisher
    dst_camera_publisher_ = it.advertiseCamera("dst_camera", 1, true);

    //
    surface_subscriber_ = nh.subscribe("surface", 1, &ImageReprojection::onSurfaceRecieved, this);

    // start the source image subscriber
    {
      const bool background(pnh.param("map_update/background", false));
      if (background) {
        const ros::Rate rate(pnh.param("map_update/frequency", 5.));
        timer_ = nh.createTimer(rate, &ImageReprojection::onMapUpdateEvent, this);
      }
      // TODO: launch multiple subscribers
      const image_transport::TransportHints default_hints;
      src_camera_subscriber_ = it.subscribeCamera(
          "src_camera0", 1,
          background ? &ImageReprojection::onSrcRecievedFast : &ImageReprojection::onSrcRecieved,
          this,
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

  void onSrcRecieved(const sensor_msgs::ImageConstPtr &ros_src,
                     const sensor_msgs::CameraInfoConstPtr &src_camera_info) {
    try {
      // do nothing if there is no node that subscribes this node
      if (dst_camera_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // convert the ROS source image to an opencv image
      cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

      //
      const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
      CV_Assert(dst_camera_info);

      // prepare the destination image
      cv_bridge::CvImage cv_dst;
      cv_dst.header.seq = cv_src->header.seq;
      cv_dst.header.stamp = cv_src->header.stamp;
      cv_dst.header.frame_id = dst_camera_info->header.frame_id;
      cv_dst.encoding = cv_src->encoding;

      // update mapping between the source and destination images
      // (this elapses 90+% of excecution time of this function
      //  and can be done without receiving the source image.
      //  this is why the fast mode is requred for some application.)
      updateMap();

      // fill the destination image by remapping the source image
      remap(cv_src->image, cv_dst.image);

      // publish the destination image
      dst_camera_publisher_.publish(cv_dst.toImageMsg(), dst_camera_info);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onSrcRecieved: " << ex.what());
    }
  }

  void onSrcRecievedFast(const sensor_msgs::ImageConstPtr &ros_src,
                         const sensor_msgs::CameraInfoConstPtr &src_camera_info) {
    try {
      // do nothing if there is no node that subscribes this node
      if (dst_camera_publisher_.getNumSubscribers() == 0) {
        return;
      }

      // convert the ROS source image to an opencv image
      cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

      //
      const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
      CV_Assert(dst_camera_info);

      // prepare the destination image
      cv_bridge::CvImage cv_dst;
      cv_dst.header.seq = cv_src->header.seq;
      cv_dst.header.stamp = cv_src->header.stamp;
      cv_dst.header.frame_id = dst_camera_info->header.frame_id;
      cv_dst.encoding = cv_src->encoding;

      // fill the destination image by remapping the source image
      remap(cv_src->image, cv_dst.image);

      // publish the destination image
      dst_camera_publisher_.publish(cv_dst.toImageMsg(), dst_camera_info);
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("onSrcRecievedFast: " << ex.what());
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
    // calculate mapping from destination image coord to destination object coord
    cv::Mat ray_directions;
    cv::Mat updated_mask(cv::Mat::ones(seed_map_.size(), CV_8UC1));
    dst_camera_model_->projectPixelTo3dRay(seed_map_, ray_directions, updated_mask);

    // transform rays into surface coordinate frame
    cv::Vec3f ray_origin(0., 0., 0.);
    {
      const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
      CV_Assert(dst_camera_info);
      tf::StampedTransform dst2surface;
      tf_listener_->lookupTransform(dst_camera_info->header.frame_id, surface_model_->getFrameId(),
                                    ros::Time(0), dst2surface);
      transform(ray_origin, dst2surface);
      transform(ray_directions, dst2surface, updated_mask);
    }

    // calculate mapping to source object coord
    cv::Mat intersections;
    surface_model_->intersection(ray_origin, ray_directions, intersections, updated_mask);

    // transform intersection points into camera frame
    {
      const sensor_msgs::CameraInfoConstPtr src_camera_info(src_camera_model_->toCameraInfo());
      CV_Assert(src_camera_info);
      tf::StampedTransform surface2src;
      tf_listener_->lookupTransform(surface_model_->getFrameId(), src_camera_info->header.frame_id,
                                    ros::Time(0), surface2src);
      transform(intersections, surface2src, updated_mask);
    }

    // calculate mapping to source image coord that is the final map
    cv::Mat updated_map;
    src_camera_model_->project3dToPixel(intersections, updated_map, updated_mask);

    // write updated mapping between source and destination images
    {
      boost::lock_guard< boost::mutex > lock(mutex_);
      cv::resize(updated_map, map_, map_.size());
      cv::resize(updated_mask, mask_, mask_.size());
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
  boost::scoped_ptr< camera_info_manager::CameraInfoManager > dst_camera_info_manager_;
  SurfaceModelPtr surface_model_;

  // subscriber for source image and publisher for destination image
  image_transport::CameraSubscriber src_camera_subscriber_;
  ros::Subscriber surface_subscriber_;
  image_transport::CameraPublisher dst_camera_publisher_;
  ros::Timer timer_;

  // transformer
  boost::scoped_ptr< tf::TransformListener > tf_listener_;

  // parameters
  cv::Mat seed_map_;

  // mapping between source and destination images
  boost::mutex mutex_;
  cv::Mat map_;
  cv::Mat mask_;
};
} // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
