#ifndef IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP
#define IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP

#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_reprojection/projection_interface.hpp>
#include <image_reprojection/transform_interface.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/rate.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>
#include <utility_headers/param.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection {

class ImageReprojection : public nodelet::Nodelet {
   public:
    ImageReprojection()
        : projection_loader_("image_reprojection", "image_reprojection::ProjectionInterface"),
          transform_loader_("image_reprojection", "image_reprojection::TransformInterface") {}

    virtual ~ImageReprojection() {
        // stop using plugins
        subscriber_.shutdown();
        timer_.stop();

        // destroy all plugins before destroying loaders
        src_projection_.reset();
        dst_projection_.reset();
        transform_.reset();
    }

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        // get node handles
        const ros::NodeHandle &nh(getNodeHandle());
        const ros::NodeHandle &pnh(getPrivateNodeHandle());

        // load plugins
        {
            std::string type;
            uhp::getRequired(pnh, "src_projection/type", type);
            src_projection_ = projection_loader_.createInstance(type);
            src_projection_->init(pnh.resolveName("src_projection"), ros::M_string(), getMyArgv());
        }
        {
            std::string type;
            uhp::getRequired(pnh, "dst_projection/type", type);
            dst_projection_ = projection_loader_.createInstance(type);
            dst_projection_->init(pnh.resolveName("dst_projection"), ros::M_string(), getMyArgv());
        }
        {
            std::string type;
            uhp::getRequired(pnh, "transform/type", type);
            transform_ = transform_loader_.createInstance(type);
            transform_->init(pnh.resolveName("transform"), ros::M_string(), getMyArgv());
        }

        // init mapping between source and destination images
        {
            // load sizes of the initial and final maps (must be initial <= final)
            const cv::Vec2i size_dst(uhp::param(pnh, "dst_image/size", cv::Vec2i(500, 500)));
            const cv::Vec2i size_seed(uhp::param(pnh, "map_update/size", cv::Vec2i(250, 250)));
            CV_Assert(size_dst(0) >= size_seed(0) && size_dst(1) >= size_seed(0));

            // init final map whose size is same as the destination image
            map_.create(size_dst(1), size_dst(0), CV_32FC2);
            for (int x = 0; x < map_.size().width; ++x) {
                for (int y = 0; y < map_.size().height; ++y) {
                    map_.at<cv::Point2f>(y, x) = cv::Point2f(x + 0.5, y + 0.5);
                }
            }
            mask_ = cv::Mat::ones(map_.size(), CV_8UC1);

            // init the seed map by resizing the final map
            cv::resize(map_, seed_map_, cv::Size(size_seed(0), size_seed(1)));
        }

        // start image reprojection
        {
            image_transport::ImageTransport it(nh);

            // setup the dstination image publisher
            publisher_ = it.advertise(
                uhp::param<std::string>(pnh, "dst_image/topic", "reprojected_image"), 1);
            frame_id_ = uhp::param<std::string>(pnh, "dst_image/fram_id", "reprojected_camera");

            // setup the source image subscriber
            const std::string topic_src(uhp::param<std::string>(pnh, "src_image/topic", "image"));
            const std::string transport_src(
                uhp::param<std::string>(pnh, "src_image/transport", "raw"));
            if (uhp::param(pnh, "map_update/background", false)) {
                // background mode setup
                timer_ = nh.createTimer(ros::Rate(uhp::param(pnh, "map_update/frequency", 5.)),
                                        &ImageReprojection::onMapUpdateEvent, this);
                subscriber_ = it.subscribe(topic_src, 1, &ImageReprojection::onSrcRecievedFast,
                                           this, transport_src);
            } else {
                // normal mode setup
                subscriber_ = it.subscribe(topic_src, 1, &ImageReprojection::onSrcRecieved, this,
                                           transport_src);
            }
        }
    }

    void onSrcRecieved(const sensor_msgs::ImageConstPtr &ros_src) {
        try {
            // do nothing if there is no node that subscribes this node
            if (publisher_.getNumSubscribers() == 0) {
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
            publisher_.publish(cv_dst.toImageMsg());
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("onSrcRecieved: " << ex.what());
        }
    }

    void onSrcRecievedFast(const sensor_msgs::ImageConstPtr &ros_src) {
        try {
            // do nothing if there is no node that subscribes this node
            if (publisher_.getNumSubscribers() == 0) {
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
            publisher_.publish(cv_dst.toImageMsg());
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
            cv::resize(map_src, map_, map_.size());
            cv::resize(cv::min(mask_dst, mask_src), mask_, mask_.size());
        }
    }

    void remap(const cv::Mat &src, cv::Mat &dst) {
        // read mapping between source and destination images
        cv::Mat map;
        cv::Mat mask;
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            map = map_.clone();
            mask = mask_.clone();
        }

        // remap the source image to a temp image
        cv::Mat tmp;
        cv::remap(src, tmp, map, cv::noArray(), cv::INTER_LINEAR);

        // copy unmasked pixels of the temp image to the destination image
        dst = cv::Mat::zeros(map.size(), src.type());
        tmp.copyTo(dst, mask);
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
    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    ros::Timer timer_;

    // parameters
    std::string frame_id_;
    cv::Mat seed_map_;

    // mapping between source and destination images
    boost::mutex mutex_;
    cv::Mat map_;
    cv::Mat mask_;
};
}

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
