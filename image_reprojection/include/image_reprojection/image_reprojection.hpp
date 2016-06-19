#ifndef IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP
#define IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP

#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_reprojection/projection_interface.hpp>
#include <image_reprojection/transform_interface.hpp>
#include <image_transport/image_transport.h>
#include <interprocess_utilities/image_transport_publisher.hpp>
#include <interprocess_utilities/image_transport_subscriber.hpp>
#include <interprocess_utilities/interprocess_publisher.hpp>
#include <interprocess_utilities/interprocess_subscriber.hpp>
#include <interprocess_utilities/publisher_interface.hpp>
#include <interprocess_utilities/subscriber_interface.hpp>
#include <nodelet/nodelet.h>
#include <param_utilities/param_utilities.hpp>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection {

class ImageReprojection : public nodelet::Nodelet {
   private:
    typedef interprocess_utilities::PublisherInterface<sensor_msgs::Image>::Ptr PublisherPtr;
    typedef interprocess_utilities::SubscriberInterface<sensor_msgs::Image>::Ptr SubscriberPtr;

   public:
    ImageReprojection()
        : projection_loader_("image_reprojection", "image_reprojection::ProjectionInterface"),
          transform_loader_("image_reprojection", "image_reprojection::TransformInterface") {}

    virtual ~ImageReprojection() {
        // stop using plugins
        subscriber_.reset();
        timer_.stop();

        // destroy all plugins before destroying loaders
        src_projection_.reset();
        dst_projection_.reset();
        transform_.reset();
    }

   private:
    virtual void onInit() {
        namespace iu = interprocess_utilities;
        namespace pu = param_utilities;

        // get node handles
        const ros::NodeHandle &nh(getNodeHandle());
        const ros::NodeHandle &pnh(getPrivateNodeHandle());

        // load plugins
        {
            std::string type;
            pu::getRequired(pnh, "src_projection/type", type);
            src_projection_ = projection_loader_.createInstance(type);
            src_projection_->init(pnh.resolveName("src_projection"), ros::M_string(), getMyArgv());
        }
        {
            std::string type;
            pu::getRequired(pnh, "dst_projection/type", type);
            dst_projection_ = projection_loader_.createInstance(type);
            dst_projection_->init(pnh.resolveName("dst_projection"), ros::M_string(), getMyArgv());
        }
        {
            std::string type;
            pu::getRequired(pnh, "transform/type", type);
            transform_ = transform_loader_.createInstance(type);
            transform_->init(pnh.resolveName("transform"), ros::M_string(), getMyArgv());
        }

        // init mapping between source and destination images
        {
            // load sizes of the initial and final maps (must be initial <= final)
            const cv::Vec2i size_dst(pu::param(pnh, "dst_image/size", cv::Vec2i(500, 500)));
            const cv::Vec2i size_seed(pu::param(pnh, "map_update/size", cv::Vec2i(250, 250)));
            CV_Assert(size_dst(0) >= size_seed(0) && size_dst(1) >= size_seed(0));

            // init final map whose size is same as the destination image
            // (then convert maps to integer for faster remapping)
            cv::Mat map(size_dst(1), size_dst(0), CV_32FC2);
            for (int x = 0; x < map.size().width; ++x) {
                for (int y = 0; y < map.size().height; ++y) {
                    map.at<cv::Point2f>(y, x) = cv::Point2f(x + 0.5, y + 0.5);
                }
            }
            cv::convertMaps(map, cv::noArray(), map1_, map2_, CV_16SC2);
            mask_ = cv::Mat::ones(map.size(), CV_8UC1);

            // init the seed map by resizing the final map
            cv::resize(map, seed_map_, cv::Size(size_seed(0), size_seed(1)));
        }

        // setup the destination image publisher
        {
            const std::string topic(
                pu::param<std::string>(pnh, "dst_image/topic", "reprojected_image"));
            const bool use_interprocess(pu::param(pnh, "dst_image/use_interprocess", false));
            frame_id_ = pu::param<std::string>(pnh, "dst_image/frame_id", "reprojected_camera");
            if (use_interprocess) {
                publisher_ = iu::advertiseInterprocess<sensor_msgs::Image>(nh.resolveName(topic));
            } else {
                publisher_ =
                    iu::advertiseImageTransport(image_transport::ImageTransport(nh), topic);
            }
            CV_Assert(publisher_);
        }

        // start the source image subscriber
        {
            const bool background(pu::param(pnh, "map_update/background", false));
            const std::string topic(pu::param<std::string>(pnh, "src_image/topic", "image"));
            const bool use_interprocess(pu::param(pnh, "src_image/use_interprocess", false));
            if (background) {
                const ros::Rate rate(pu::param(pnh, "map_update/frequency", 5.));
                timer_ = nh.createTimer(rate, &ImageReprojection::onMapUpdateEvent, this);
            }
            if (use_interprocess) {
                subscriber_ = iu::subscribeInterprocess<sensor_msgs::Image>(
                    nh.resolveName(topic), background ? &ImageReprojection::onSrcRecievedFast
                                                      : &ImageReprojection::onSrcRecieved,
                    this);
            } else {
                const std::string transport(
                    pu::param<std::string>(pnh, "src_image/transport", "raw"));
                subscriber_ =
                    iu::subscribeImageTransport(image_transport::ImageTransport(nh), topic,
                                                background ? &ImageReprojection::onSrcRecievedFast
                                                           : &ImageReprojection::onSrcRecieved,
                                                this, transport);
            }
            CV_Assert(subscriber_);
        }
    }

    void onSrcRecieved(const sensor_msgs::ImageConstPtr &ros_src) {
        try {
            // do nothing if there is no node that subscribes this node
            if (publisher_->getNumSubscribers() == 0) {
                return;
            }

            // convert the ROS source image to an opencv image
            cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

            // prepare the destination image
            cv_bridge::CvImage cv_dst;
            cv_dst.header.seq = cv_src->header.seq;
            cv_dst.header.stamp = cv_src->header.stamp;
            cv_dst.header.frame_id = frame_id_;
            cv_dst.encoding = cv_src->encoding;

            // update mapping between the source and destination images
            // (this elapses 90+% of excecution time of this function
            //  and can be done without receiving the source image.
            //  this is why the fast mode is requred for some application.)
            updateMap();

            // fill the destination image by remapping the source image
            remap(cv_src->image, cv_dst.image);

            // publish the destination image
            publisher_->publish(*cv_dst.toImageMsg());
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("onSrcRecieved: " << ex.what());
        }
    }

    void onSrcRecievedFast(const sensor_msgs::ImageConstPtr &ros_src) {
        try {
            // do nothing if there is no node that subscribes this node
            if (publisher_->getNumSubscribers() == 0) {
                return;
            }

            // convert the ROS source image to an opencv image
            cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

            // prepare the destination image
            cv_bridge::CvImage cv_dst;
            cv_dst.header.seq = cv_src->header.seq;
            cv_dst.header.stamp = cv_src->header.stamp;
            cv_dst.header.frame_id = frame_id_;
            cv_dst.encoding = cv_src->encoding;

            // fill the destination image by remapping the source image
            remap(cv_src->image, cv_dst.image);

            // publish the destination image
            publisher_->publish(*cv_dst.toImageMsg());
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
        cv::Mat map_dst;
        cv::Mat mask_dst;
        dst_projection_->reproject(seed_map_, map_dst, mask_dst);

        // calculate mapping to source object coord
        cv::Mat map_trans;
        transform_->inverseTransform(map_dst, map_trans);

        // calculate mapping to source image coord that is the final map
        cv::Mat map_src;
        cv::Mat mask_src;
        src_projection_->project(map_trans, map_src, mask_src);

        // write updated mapping between source and destination images
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            cv::Mat map;
            cv::resize(map_src, map, map1_.size());
            cv::convertMaps(map, cv::noArray(), map1_, map2_, CV_16SC2);
            cv::resize(cv::min(mask_dst, mask_src), mask_, mask_.size());
        }
    }

    void remap(const cv::Mat &src, cv::Mat &dst) {
        boost::lock_guard<boost::mutex> lock(mutex_);

        // remap the source image to a temp image
        cv::Mat tmp;
        cv::remap(src, tmp, map1_, map2_, cv::INTER_LINEAR);

        // copy unmasked pixels of the temp image to the destination image
        dst = cv::Mat::zeros(map1_.size(), src.type());
        tmp.copyTo(dst, mask_);
    }

   private:
    // plugin loaders
    pluginlib::ClassLoader<ProjectionInterface> projection_loader_;
    pluginlib::ClassLoader<TransformInterface> transform_loader_;

    // plugins
    ProjectionInterfacePtr src_projection_;
    ProjectionInterfacePtr dst_projection_;
    TransformInterfacePtr transform_;

    // subscriber for source image and publisher for destination image
    SubscriberPtr subscriber_;
    PublisherPtr publisher_;
    ros::Timer timer_;

    // parameters
    std::string frame_id_;
    cv::Mat seed_map_;

    // mapping between source and destination images
    boost::mutex mutex_;
    cv::Mat map1_, map2_;
    cv::Mat mask_;
};
}

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
