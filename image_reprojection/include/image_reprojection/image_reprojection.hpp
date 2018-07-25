#ifndef IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP
#define IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP

#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>
#include <image_reprojection/projection_interface.hpp>
#include <image_reprojection/transform_interface.hpp>
#include <param_utilities/param_utilities.hpp>

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
        namespace pu = param_utilities;

        // get node handles
        const ros::NodeHandle &nh(getNodeHandle());
        const ros::NodeHandle &pnh(getPrivateNodeHandle());
        image_transport::ImageTransport it(nh);

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
            const cv::Size size_dst(pu::param(pnh, "dst_image/size", cv::Size(500, 500)));
            const cv::Size size_seed(pu::param(pnh, "map_update/size", cv::Size(250, 250)));
            CV_Assert(size_dst.width >= size_seed.width && size_dst.height >= size_seed.height);

            // init final map whose size is same as the destination image
            map_.create(size_dst, CV_32FC2);
            for (int x = 0; x < map_.size().width; ++x) {
                for (int y = 0; y < map_.size().height; ++y) {
                    map_.at<cv::Point2f>(y, x) = cv::Point2f(x + 0.5, y + 0.5);
                }
            }
            mask_ = cv::Mat::ones(map_.size(), CV_8UC1);

            // init the seed map by resizing the final map
            cv::resize(map_, seed_map_, size_seed);
        }

        // setup the destination image publisher
        {
            frame_id_ = pu::param<std::string>(pnh, "dst_image/frame_id", "reprojected_camera");
            publisher_ = it.advertiseCamera("virtual_camera", 1, true);
            CV_Assert(publisher_);
        }

        // start the source image subscriber
        {
            const bool background(pu::param(pnh, "map_update/background", false));
            if (background) {
                const ros::Rate rate(pu::param(pnh, "map_update/frequency", 5.));
                timer_ = nh.createTimer(rate, &ImageReprojection::onMapUpdateEvent, this);
            }
            // TODO: launch multiple subscribers
            subscriber_ = it.subscribeCamera("camera0", 1,
                                             background ? &ImageReprojection::onSrcRecievedFast
                                                        : &ImageReprojection::onSrcRecieved,
                                             this);
            CV_Assert(subscriber_);
        }
    }

    void onSrcRecieved(const sensor_msgs::ImageConstPtr &ros_src,
                       const sensor_msgs::CameraInfoConstPtr &) {
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
            // publisher_.publish(*cv_dst.toImageMsg(), camera_info);
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("onSrcRecieved: " << ex.what());
        }
    }

    void onSrcRecievedFast(const sensor_msgs::ImageConstPtr &ros_src,
                           const sensor_msgs::CameraInfoConstPtr &) {
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
            // publisher_.publish(*cv_dst.toImageMsg(), camera_info);
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
        boost::lock_guard<boost::mutex> lock(mutex_);

        // remap the source image to a temp image
        cv::Mat tmp;
        cv::remap(src, tmp, map_, cv::noArray(), cv::INTER_LINEAR);

        // copy unmasked pixels of the temp image to the destination image
        dst = cv::Mat::zeros(map_.size(), src.type());
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
    image_transport::CameraSubscriber subscriber_;
    image_transport::CameraPublisher publisher_;
    ros::Timer timer_;

    // parameters
    std::string frame_id_;
    cv::Mat seed_map_;

    // mapping between source and destination images
    boost::mutex mutex_;
    cv::Mat map_;
    cv::Mat mask_;
};
}  // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
