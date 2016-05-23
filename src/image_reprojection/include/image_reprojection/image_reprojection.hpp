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

        // load parameters
        {
            boost::array<int, 2> size = {500, 500};
            uhp::get(pnh, "image_size_dst", size);
            image_points_dst_.create(size[1], size[0], CV_32FC2);
            for (int x = 0; x < size[0]; ++x) {
                for (int y = 0; y < size[1]; ++y) {
                    cv::Point2f &dst(image_points_dst_.at<cv::Point2f>(y, x));
                    dst.x = x + 0.5;
                    dst.y = y + 0.5;
                }
            }
        }
        frame_dst_ = uhp::param<std::string>(pnh, "frame_dst", "reprojected_camera");

        // start image reprojection
        {
            image_transport::ImageTransport it(nh);
            publisher_ =
                it.advertise(uhp::param<std::string>(pnh, "topic_dst", "reprojected_image"), 1);
            if (uhp::param(pnh, "use_fast_mode", false)) {
                // fast mode setup
                timer_ = nh.createTimer(ros::Rate(uhp::param(pnh, "map_update_frequency", 5.)),
                                        &ImageReprojection::onMapUpdateEvent, this);
                subscriber_ = it.subscribe(uhp::param<std::string>(pnh, "topic_src", "image"), 1,
                                           &ImageReprojection::onSrcRecievedFast, this,
                                           uhp::param<std::string>(pnh, "transport_src", "raw"));
            } else {
                // normal mode setup
                subscriber_ = it.subscribe(uhp::param<std::string>(pnh, "topic_src", "image"), 1,
                                           &ImageReprojection::onSrcRecieved, this,
                                           uhp::param<std::string>(pnh, "transport_src", "raw"));
            }
        }
    }

    void onSrcRecieved(const sensor_msgs::ImageConstPtr &ros_src) {
        try {
            // convert the ROS source image to an opencv image
            cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

            // prepare the destination image
            cv_bridge::CvImage cv_dst;
            cv_dst.header.seq = cv_src->header.seq;
            cv_dst.header.stamp = cv_src->header.stamp;
            cv_dst.header.frame_id = frame_dst_;
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
            // convert the ROS source image to an opencv image
            cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

            // prepare the destination image
            cv_bridge::CvImage cv_dst;
            cv_dst.header.seq = cv_src->header.seq;
            cv_dst.header.stamp = cv_src->header.stamp;
            cv_dst.header.frame_id = frame_dst_;
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
        cv::Mat object_points_dst;
        cv::Mat mask_dst;
        dst_projection_->reproject(image_points_dst_, object_points_dst, mask_dst);

        // calculate mapping to source object coord
        cv::Mat object_points_src;
        transform_->inverseTransform(object_points_dst, object_points_src);

        // calculate mapping to source image coord that is the final map
        cv::Mat image_points_src;
        cv::Mat mask_src;
        src_projection_->project(object_points_src, image_points_src, mask_src);

        // write updated mapping between source and destination images
        {
            boost::lock_guard<boost::mutex> lock(mutex_);
            map_ = image_points_src.clone();
            mask_ = cv::max(mask_src, mask_dst);
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
    std::string frame_dst_;
    cv::Mat image_points_dst_;

    // mapping between source and destination images
    boost::mutex mutex_;
    cv::Mat map_;
    cv::Mat mask_;
};
}

#endif /* IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP */
