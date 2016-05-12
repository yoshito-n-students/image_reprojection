#ifndef _IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP_
#define _IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP_

#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_reprojection/projection_plugin.hpp>
#include <image_reprojection/transform_plugin.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/param.h>
#include <sensor_msgs/Image.h>
#include <utility_headers/param.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection {

class ImageReprojection : public nodelet::Nodelet {
   public:
    ImageReprojection()
        : projection_loader_("image_reprojection", "image_reprojection::ProjectionPlugin"),
          transform_loader_("image_reprojection", "image_reprojection::TransformPlugin") {}

    virtual ~ImageReprojection() {
        // destroy all classes from plugins before destroying loaders
        src_projection_.reset();
        dst_projection_.reset();
        transform_.reset();
    }

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        const ros::NodeHandle &nh(getNodeHandle());
        const ros::NodeHandle &pnh(getPrivateNodeHandle());

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

        {
            boost::array<int, 2> size = {1280, 720};
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

        {
            image_transport::ImageTransport it(nh);
            publisher_ =
                it.advertise(uhp::param<std::string>(pnh, "topic_dst", "reprojected_image"), 1);
            subscriber_ = it.subscribe(uhp::param<std::string>(pnh, "topic_src", "image"), 1,
                                       &ImageReprojection::onSrcRecieved, this,
                                       uhp::param<std::string>(pnh, "transport_src", "raw"));
        }
    }

    void onSrcRecieved(const sensor_msgs::ImageConstPtr &ros_src) {
        try {
            cv_bridge::CvImageConstPtr cv_src(cv_bridge::toCvShare(ros_src));

            cv_bridge::CvImage cv_dst;
            cv_dst.header.seq = cv_src->header.seq;
            cv_dst.header.stamp = cv_src->header.stamp;
            cv_dst.header.frame_id = frame_dst_;
            cv_dst.encoding = cv_src->encoding;

            cv::Mat object_points_dst;
            cv::Mat mask_dst;
            dst_projection_->reproject(image_points_dst_, object_points_dst, mask_dst);

            cv::Mat object_points_src;
            transform_->inverseTransform(object_points_dst, object_points_src);

            cv::Mat image_points_src;
            cv::Mat mask_src;
            src_projection_->project(object_points_src, image_points_src, mask_src);

            remap(cv_src->image, cv_dst.image, image_points_src, cv::max(mask_src, mask_dst));

            publisher_.publish(cv_dst.toImageMsg());
        } catch (const std::runtime_error &ex) {
            ROS_ERROR_STREAM(ex.what());
        }
    }

    static void remap(const cv::Mat &src, cv::Mat &dst, const cv::Mat &map, const cv::Mat &mask) {
        // remap the source image to a temp image
        cv::Mat tmp;
        cv::remap(src, tmp, map, cv::noArray(), cv::INTER_LINEAR);

        // copy unmasked pixels of the temp image to the destination image
        dst = cv::Mat::zeros(map.size(), src.type());
        tmp.copyTo(dst, mask);
    }

   private:
    pluginlib::ClassLoader<ProjectionPlugin> projection_loader_;
    pluginlib::ClassLoader<TransformPlugin> transform_loader_;

    image_transport::Subscriber subscriber_;
    image_transport::Publisher publisher_;
    std::string frame_dst_;

    ProjectionPluginPtr src_projection_;
    ProjectionPluginPtr dst_projection_;
    TransformPluginPtr transform_;

    cv::Mat image_points_dst_;
};
}

#endif /* _IMAGE_REPROJECTION_IMAGE_REPROJECTION_HPP_ */
