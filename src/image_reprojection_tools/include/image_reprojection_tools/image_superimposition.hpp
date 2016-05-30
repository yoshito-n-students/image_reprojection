#ifndef IMAGE_REPROJECTION_TOOS_IMAGE_SUPERIMPOSITION_HPP
#define IMAGE_REPROJECTION_TOOS_IMAGE_SUPERIMPOSITION_HPP

#include <cmath>
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
        cv::Matx33f perspective;
        cv::Mat mask;
        double transparency;
        // mutable
        cv::Mat image;
        mutable cv::Mutex mutex;  // use mutable tag to lock for read-only use
    };

   public:
    ImageSuperimposition() {}

    virtual ~ImageSuperimposition() {
        // shutdown all subscribers before the publisher
        // or a subscriber may call the publisher inactivated prior to the subscriber
        for (std::vector<Layer>::iterator layer = layers_.begin(); layer != layers_.end();
             ++layer) {
            layer->subscriber.shutdown();
        }
        publisher_.shutdown();
    }

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
            // copy if 'transform' param is a 2x3 (affine) or 3x3 (perspective) matrix
            layers_[i].perspective = cv::Matx33f::eye();
            if (params[i].transform.size() == 2 || params[i].transform.size() == 3) {
                for (int j = 0; j < params[i].transform.size(); ++j) {
                    for (int k = 0; k < 3; ++k) {
                        layers_[i].perspective(j, k) = params[i].transform[j][k];
                    }
                }
            }
            ROS_INFO_STREAM(layers_[i].perspective);

            // create a mask that indicates AOI (area of interest)
            if (params[i].aoi.size() < 2) {
                // 'AOI' param that has 0 or 1 point: default AOI: whole area of the result image
                layers_[i].mask = cv::Mat::ones(size_, CV_8UC1);
            } else {
                std::vector<cv::Vec2f> aoi;
                if (params[i].aoi.size() == 2) {
                    // 'AOI' param that has 2 points: rectangle AOI
                    aoi.push_back(params[i].aoi[0]);
                    aoi.push_back(cv::Vec2f(params[i].aoi[0][0], params[i].aoi[1][1]));
                    aoi.push_back(params[i].aoi[1]);
                    aoi.push_back(cv::Vec2f(params[i].aoi[1][0], params[i].aoi[0][1]));
                } else {
                    // 'AOI' param that has 3+ points: polygon AOI
                    aoi = params[i].aoi;
                }

                // apply the perspective transform to the AOI
                std::vector<cv::Vec2f> transformed_aoi;
                cv::perspectiveTransform(aoi, transformed_aoi, layers_[i].perspective);

                // create a mask as the filled AOI
                layers_[i].mask = cv::Mat::zeros(size_, CV_8UC1);
                {
                    // convert floating-points to integers
                    // because cv::perspectiveTransform supports the former only
                    // and cv::fillPoly supports the latter only
                    std::vector<std::vector<cv::Point> > polygon(1);
                    for (int j = 0; j < transformed_aoi.size(); ++j) {
                        // standard c++ lib does not contain 'std::round', but c contains 'round'
                        polygon[0].push_back(cv::Point(::round(transformed_aoi[j][0]), ::round(transformed_aoi[j][1])));
                    }
                    cv::fillPoly(layers_[i].mask, polygon, 1);
                }
            }

            // copy cliped transparency [0,1]
            layers_[i].transparency = std::min(std::max(params[i].transparency, 0.), 1.);

            // create a brank layer image
            layers_[i].image = cv::Mat::zeros(size_, cv_bridge::getCvType(encoding_));

            // launch a subscriber
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
            // just save this image message
            updateLayer(msg, layer);
        } catch (const std::exception &ex) {
            ROS_ERROR_STREAM("onImageReceived: " << layer.subscriber.getTopic() << ": "
                                                 << ex.what());
        }
    }

    void onPrimaryImageReceived(const sensor_msgs::ImageConstPtr &msg, Layer &layer) {
        try {
            // first save this image message
            updateLayer(msg, layer);

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
            ROS_ERROR_STREAM("onPrimaryImageReceived: " << layer.subscriber.getTopic() << ": "
                                                        << ex.what());
        }
    }

    void updateLayer(const sensor_msgs::ImageConstPtr &msg, Layer &layer) {
        // ROS -> opencv2
        const cv_bridge::CvImageConstPtr image(cv_bridge::toCvShare(msg, encoding_));

        // do perspective transform and save the transformed image
        {
            cv::AutoLock lock(layer.mutex);
            cv::warpPerspective(image->image, layer.image, layer.perspective, size_);
        }
    }

    cv::Mat superimposeLayers() const {
        // stack layers from bottom to top
        cv::Mat image(cv::Mat::zeros(size_, cv_bridge::getCvType(encoding_)));
        for (std::vector<Layer>::const_reverse_iterator layer = layers_.rbegin();
             layer != layers_.rend(); ++layer) {
            cv::Mat this_image(layer->transparency * image);
            {
                cv::AutoLock lock(layer->mutex);
                this_image += (1. - layer->transparency) * layer->image;
            }
            this_image.copyTo(image, layer->mask);
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