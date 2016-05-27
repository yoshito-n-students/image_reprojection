#ifndef IMAGE_REPROJECTION_TOOS_IMAGE_SUPERIMPOSITION_HPP
#define IMAGE_REPROJECTION_TOOS_IMAGE_SUPERIMPOSITION_HPP

#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_reprojection_tools/image_superimposition_params.hpp>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <image_transport/subscriber.h>
#include <nodelet/nodelet.h>
#include <utility_headers/param.hpp>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>

#include <boost/bind.hpp>
#include <boost/cstdint.hpp>
#include <boost/ref.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection_tools {

class ImageSuperimposition : public nodelet::Nodelet {
   private:
    struct Layer {
        // constant
        image_transport::Subscriber subscriber;
        cv::Matx23f affine_matrix;
        cv::Size size;
        cv::Point origin;
        // mutable
        cv::Mat image;
        mutable cv::Mutex mutex;  // use mutable tag to lock for read-only use
    };

   public:
    ImageSuperimposition() {}

    virtual ~ImageSuperimposition() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        const ros::NodeHandle &nh(getNodeHandle());
        const ros::NodeHandle &pnh(getPrivateNodeHandle());

        // load parameters
        frame_id_ = uhp::param<std::string>(pnh, "frame_id", "superimpose_images");
        encoding_ = uhp::param<std::string>(pnh, "encoding", "bgr8");
        {
            const cv::Vec2i size(uhp::param(pnh, "size", cv::Vec2i(500, 500)));
            size_ = cv::Size(size(0), size(1));
        }
        std::vector<ImageSuperimpositionParams> params;
        uhp::getRequired(pnh, "layers", params);

        // setup the publisher
        image_transport::ImageTransport it(nh);
        publisher_ = it.advertise(uhp::param<std::string>(pnh, "topic", "superimposed_image"), 1);
        seq_ = 0;

        // setup workers for layers
        layers_.resize(params.size());
        for (std::size_t i = 0; i < params.size(); ++i) {
            layers_[i].affine_matrix = params[i].affine_matrix;
            layers_[i].size = params[i].size;
            layers_[i].origin = params[i].origin;
            layers_[i].image = cv::Mat::zeros(params[i].size, cv_bridge::getCvType(encoding_));
            layers_[i].subscriber = it.subscribe(
                params[i].topic, 1,
                boost::bind(params[i].primary ? &ImageSuperimposition::onPrimaryImageReceived
                                              : &ImageSuperimposition::onImageReceived,
                            this, _1, boost::ref(layers_[i])),
                ros::VoidPtr(), params[i].transport);
        }

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    void onImageReceived(const sensor_msgs::ImageConstPtr &msg, Layer &layer) {
        try {
            // just seve this image message
            save(msg, layer);
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("onImageReceived: " << ex.what());
        }
    }

    void onPrimaryImageReceived(const sensor_msgs::ImageConstPtr &msg, Layer &layer) {
        try {
            // first save this image message
            save(msg, layer);

            // do nothing further if there is no node that subscribes this node
            if (publisher_.getNumSubscribers() == 0) {
                return;
            }

            // prepare the destination image
            cv_bridge::CvImage dst;
            dst.header.stamp = ros::Time::now();
            dst.header.frame_id = frame_id_;
            dst.encoding = encoding_;
            dst.image = superimposeLayers();

            // finally publish the superimposed image
            {
                cv::AutoLock lock(mutex_);
                dst.header.seq = (seq_++);
                publisher_.publish(dst.toImageMsg());
            }
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("onPrimaryImageReceived: " << ex.what());
        }
    }

    void save(const sensor_msgs::ImageConstPtr &msg, Layer &layer) {
        // ROS -> opencv2
        cv_bridge::CvImageConstPtr image(cv_bridge::toCvShare(msg, encoding_));

        // do affine transform and save the transformed image
        {
            cv::AutoLock lock(layer.mutex);
            cv::warpAffine(image->image, layer.image, layer.affine_matrix, layer.size);
        }
    }

    cv::Mat superimposeLayers() {
        // stack layers from bottom to top
        cv::Mat image(cv::Mat::zeros(size_, cv_bridge::getCvType(encoding_)));
        for (std::vector<Layer>::const_reverse_iterator layer = layers_.rbegin();
             layer != layers_.rend(); ++layer) {
            cv::AutoLock lock(layer->mutex);
            layer->image.copyTo(image(cv::Rect(layer->origin, layer->size)));
        }
        return image;
    }

   private:
    // parameters
    std::string frame_id_;
    std::string encoding_;
    cv::Size size_;

    // image layers
    std::vector<Layer> layers_;

    // image publisher
    image_transport::Publisher publisher_;
    boost::uint32_t seq_;
    cv::Mutex mutex_;
};
}

#endif  // IMAGE_REPROJECTION_TOOS_IMAGE_SUPERIMPOSITION_HPP