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
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/service_server.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <topic_tools/shape_shifter.h>

#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

namespace image_reprojection {

class ImageReprojection : public nodelet::Nodelet {
public:
  ImageReprojection()
      : camera_model_loader_("image_reprojection", "image_reprojection::CameraModel"),
        surface_model_loader_("image_reprojection", "image_reprojection::SurfaceModel") {}

  virtual ~ImageReprojection() {
    // stop using plugins
    src_camera_subscribers_.clear();
    surface_subscriber_.shutdown();
    dst_camera_info_server_.shutdown();
    dst_camera_timer_.stop();
    map_timer_.stop();

    // destroy all plugins before destroying loaders
    src_camera_models_.clear();
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
    // src camera
    //

    for (int i = 0; true; ++i) {
      const std::string i_str(boost::lexical_cast< std::string >(i));
      if (!pnh.hasParam("src_camera" + i_str + "/model")) {
        break;
      }

      // load src camera model instance
      {
        std::string type;
        pnh.getParam("src_camera" + i_str + "/model", type);
        src_camera_models_.push_back(camera_model_loader_.createInstance(type));
        src_camera_models_.back()->init(pnh.resolveName("src_camera" + i_str), ros::M_string(),
                                        getMyArgv(), &getSTCallbackQueue(), &getMTCallbackQueue());
      }

      // prepare src image storage
      src_images_.push_back(cv_bridge::CvImageConstPtr());
      maps_.push_back(cv::Mat());
      masks_.push_back(cv::Mat());

      // subscribe src camera
      {
        const image_transport::TransportHints default_hints;
        src_camera_subscribers_.push_back(it.subscribeCamera(
            "src_image" + i_str, 1,
            boost::bind(&ImageReprojection::onSrcCameraRecieved, this, _1, _2, i), ros::VoidPtr(),
            image_transport::TransportHints(default_hints.getTransport(),
                                            default_hints.getRosHints(),
                                            ros::NodeHandle(pnh, "src_camera" + i_str))));
      }
    }
    CV_Assert(src_camera_models_.size() > 0);

    //
    // surface
    //

    // load surface model instance
    {
      std::string type;
      CV_Assert(pnh.getParam("surface/model", type));
      surface_model_ = surface_model_loader_.createInstance(type);
      surface_model_->init(pnh.resolveName("surface"), ros::M_string(), getMyArgv(),
                           &getSTCallbackQueue(), &getMTCallbackQueue());
    }

    // setup the surface subscriber
    surface_subscriber_ = nh.subscribe("surface", 1, &ImageReprojection::onSurfaceRecieved, this);

    //
    // dst camera and map update
    //

    // load parameters
    dst_image_encoding_ =
        pnh.param< std::string >("dst_camera/encoding", sensor_msgs::image_encodings::BGR8);
    map_binning_x_ = pnh.param("map_update/binning_x", 8);
    map_binning_y_ = pnh.param("map_update/binning_y", 8);

    // setup the coordinate frame transformer
    tf_listener_.reset(new tf::TransformListener(nh));

    // load dst camera instance
    {
      std::string type;
      CV_Assert(pnh.getParam("dst_camera/model", type));
      dst_camera_model_ = camera_model_loader_.createInstance(type);
      dst_camera_model_->init(pnh.resolveName("dst_camera"), ros::M_string(), getMyArgv(),
                              &getSTCallbackQueue(), &getMTCallbackQueue());
    }

    // init dst camera model
    {
      std::string info_file;
      CV_Assert(pnh.getParam("dst_camera/info_file", info_file));
      sensor_msgs::CameraInfo info;
      CV_Assert(camera_calibration_parsers::readCalibration(info_file, info.header.frame_id, info));
      dst_camera_model_->fromCameraInfo(info);
    }

    // setup the destination camera info updater
    dst_camera_info_server_ =
        nh.advertiseService("set_dst_camera_info", &ImageReprojection::onDstCameraInfoSet, this);

    // setup the destination image publisher
    dst_camera_publisher_ = it.advertiseCamera("dst_image", 1, true);

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

  //
  // passive event handlers
  //

  void onSrcCameraRecieved(const sensor_msgs::ImageConstPtr &src_image,
                           const sensor_msgs::CameraInfoConstPtr &src_camera_info, const int i) {
    try {
      // update source camera model
      src_camera_models_[i]->fromCameraInfo(*src_camera_info);

      // convert the ROS source image to an opencv image
      src_images_[i] = cv_bridge::toCvShare(src_image, dst_image_encoding_);
    } catch (const std::exception &ex) {
      NODELET_ERROR_STREAM("onSrcCameraRecieved(" << i << "): " << ex.what());
    }
  }

  void onSurfaceRecieved(const topic_tools::ShapeShifter::ConstPtr &surface) {
    try {
      surface_model_->update(*surface);
    } catch (const std::exception &ex) {
      NODELET_ERROR_STREAM("onSurfaceRecieved: " << ex.what());
    }
  }

  bool onDstCameraInfoSet(sensor_msgs::SetCameraInfo::Request &request,
                          sensor_msgs::SetCameraInfo::Response &response) {
    try {
      dst_camera_model_->fromCameraInfo(request.camera_info);
      response.success = true;
    } catch (const std::exception &ex) {
      NODELET_ERROR_STREAM("onDstCameraInfoSet: " << ex.what());
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
      dst_image.header.stamp = ros::Time::now();
      dst_image.header.frame_id = dst_camera_info->header.frame_id;
      dst_image.encoding = dst_image_encoding_;
      dst_image.image =
          cv::Mat::zeros(toImageSize(*dst_camera_info), cv_bridge::getCvType(dst_image_encoding_));

      if (do_update_map) {
        // update mapping between the source and destination images
        // (this elapses 90+% of excecution time of this function
        //  and can be done without receiving the source image.
        //  this is why the fast mode is requred for some application.)
        updateMap();
      }

      // fill the destination image by remapping the source image
      for (int i = 0; i < src_images_.size(); ++i) {
        if (!src_images_[i]) {
          continue;
        }
        // remap the source image to a temp image
        cv::Mat tmp;
        cv::remap(src_images_[i]->image, tmp, maps_[i], cv::noArray(), cv::INTER_LINEAR);
        // copy unmasked pixels of the temp image to the destination image
        tmp.copyTo(dst_image.image, masks_[i]);
      }

      // publish the destination image
      dst_camera_publisher_.publish(dst_image.toImageMsg(), dst_camera_info);
    } catch (const std::exception &ex) {
      NODELET_ERROR_STREAM("onDstCameraEvent: " << ex.what());
    }
  }

  void onMapUpdateEvent(const ros::TimerEvent &event) {
    try {
      updateMap();
    } catch (const std::exception &ex) {
      NODELET_ERROR_STREAM("onMapUpdateEvent: " << ex.what());
    }
  }

  void updateMap() {
    // get destination camera info
    const sensor_msgs::CameraInfoConstPtr dst_camera_info(dst_camera_model_->toCameraInfo());
    CV_Assert(dst_camera_info);

    // figure out size of destination image and shrinked map
    const cv::Size dst_image_size(toImageSize(*dst_camera_info));
    const cv::Size binned_map_size(dst_image_size.width / map_binning_x_,
                                   dst_image_size.height / map_binning_y_);

    // create initial map and mask
    cv::Mat binned_map(binned_map_size, CV_32FC2);
    for (int x = 0; x < binned_map_size.width; ++x) {
      for (int y = 0; y < binned_map_size.height; ++y) {
        binned_map.at< cv::Point2f >(y, x) =
            cv::Point2f((dst_image_size.width - 1.) * x / (binned_map_size.width - 1.),
                        (dst_image_size.height - 1.) * y / (binned_map_size.height - 1.));
      }
    }
    cv::Mat binned_mask(cv::Mat::ones(binned_map_size, CV_8UC1));

    // calculate mapping from destination pixels to ray toward surface
    cv::Mat ray_directions;
    dst_camera_model_->projectPixelTo3dRay(binned_map, ray_directions, binned_mask);

    // transform rays into surface coordinate frame
    cv::Vec3f ray_origin;
    {
      tf::StampedTransform dst2surface;
      tf_listener_->lookupTransform(/* to */ surface_model_->getFrameId(),
                                    /* from */ dst_camera_info->header.frame_id,
                                    /* at latest time */ ros::Time(0), dst2surface);
      ray_origin = transform(cv::Vec3f(0., 0., 0.), dst2surface);
      ray_directions = transform(ray_directions, dst2surface.getBasis(), binned_mask);
    }

    // calculate mapping from ray to intersection point on surface
    cv::Mat intersections;
    surface_model_->intersection(ray_origin, ray_directions, intersections, binned_mask);

    // TODO: check intersection points are visible from src camera origin using the surface model

    for (int i = 0; i < src_camera_models_.size(); ++i) {
      // transform intersection points into source camera frame
      cv::Mat intersections_i;
      cv::Mat binned_mask_i(binned_mask.clone());
      {
        const sensor_msgs::CameraInfoConstPtr src_camera_info_i(
            src_camera_models_[i]->toCameraInfo());
        CV_Assert(src_camera_info_i);
        tf::StampedTransform surface2src_i;
        tf_listener_->lookupTransform(/* to */ src_camera_info_i->header.frame_id,
                                      /* from */ surface_model_->getFrameId(),
                                      /* at latest time */ ros::Time(0), surface2src_i);
        intersections_i = transform(intersections, surface2src_i, binned_mask_i);
      }

      // calculate mapping from points on surface to source pixels
      cv::Mat binned_map_i(binned_map.clone());
      src_camera_models_[i]->project3dToPixel(intersections_i, binned_map_i, binned_mask_i);

      // inpaint invalid pixels of the shrinked map using valid pixels
      // to get better full resolution map by resizing the shrinked map
      {
        // split channels of the shrinked map because cv::inpaint() does not accept 2-channel mat
        cv::Mat binned_x_map_i, binned_y_map_i;
        cv::extractChannel(binned_map_i, binned_x_map_i, 0);
        cv::extractChannel(binned_map_i, binned_y_map_i, 1);
        // invert mask to indicate pixels to be inpainted
        cv::Mat inpaint_mask_i(cv::Mat::ones(binned_map_size, CV_8UC1));
        inpaint_mask_i.setTo(0, binned_mask_i);
        // inpaint every channel
        cv::inpaint(binned_x_map_i, inpaint_mask_i, binned_x_map_i, 1., cv::INPAINT_NS);
        cv::inpaint(binned_y_map_i, inpaint_mask_i, binned_y_map_i, 1., cv::INPAINT_NS);
        // copy inpainted channels to the shrinked map
        cv::insertChannel(binned_x_map_i, binned_map_i, 0);
        cv::insertChannel(binned_y_map_i, binned_map_i, 1);
      }

      // write updated mapping between source and destination images
      cv::resize(binned_map_i, maps_[i], dst_image_size, cv::INTER_LINEAR);
      cv::resize(binned_mask_i, masks_[i], dst_image_size, cv::INTER_NEAREST);
    }
  }

  static cv::Size toImageSize(const sensor_msgs::CameraInfo &camera_info) {
    return cv::Size((camera_info.roi.width == 0 ? camera_info.width : camera_info.roi.width) /
                        (camera_info.binning_x == 0 ? 1 : camera_info.binning_x),
                    (camera_info.roi.height == 0 ? camera_info.height : camera_info.roi.height) /
                        (camera_info.binning_y == 0 ? 1 : camera_info.binning_y));
  }

private:
  // model loaders
  pluginlib::ClassLoader< CameraModel > camera_model_loader_;
  pluginlib::ClassLoader< SurfaceModel > surface_model_loader_;

  // src cameras
  std::vector< image_transport::CameraSubscriber > src_camera_subscribers_;
  std::vector< CameraModelPtr > src_camera_models_;
  std::vector< cv_bridge::CvImageConstPtr > src_images_;

  // surface
  ros::Subscriber surface_subscriber_;
  SurfaceModelPtr surface_model_;

  // dst camera
  ros::ServiceServer dst_camera_info_server_;
  CameraModelPtr dst_camera_model_;
  image_transport::CameraPublisher dst_camera_publisher_;
  ros::Timer dst_camera_timer_;
  std::string dst_image_encoding_;

  // mapping between src and dst image pixels
  ros::Timer map_timer_;
  boost::scoped_ptr< tf::TransformListener > tf_listener_;
  int map_binning_x_;
  int map_binning_y_;
  std::vector< cv::Mat > maps_;
  std::vector< cv::Mat > masks_;
};

} // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
