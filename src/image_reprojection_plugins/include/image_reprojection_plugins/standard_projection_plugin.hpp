#ifndef _IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_PLUGIN_HPP_
#define _IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_PLUGIN_HPP_

#include <vector>

#include <image_reprojection/projection_plugin.hpp>
#include <ros/console.h>
#include <utility_headers/param.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection_plugins {

class StandardProjectionPlugin : public image_reprojection::ProjectionPlugin {
   public:
    StandardProjectionPlugin() {}

    virtual ~StandardProjectionPlugin() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        // get private node handle to access parameters
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        // load parameters
        uhp::getRequired(pnh, "camera_matrix", camera_matrix_);
        dist_coeffs_ = uhp::param(pnh, "dist_coeffs", std::vector<double>(4, 0.));

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        cv::projectPoints(src.reshape(3, src.total()), cv::Vec3d::all(0.), cv::Vec3d::all(0.),
                          camera_matrix_, dist_coeffs_, dst);
        dst = dst.reshape(2, src.size().height);

        mask.create(src.size(), CV_8UC1);
        const cv::Rect_<float> rect(cv::Point2f(0., 0.), src.size());
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                mask.at<unsigned char>(y, x) = rect.contains(dst.at<cv::Point2f>(y, x)) ? 1 : 0;
            }
        }
    }

    virtual void onReproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        // 2D image points -> 2D object points
        cv::Mat dst2d;
        cv::undistortPoints(src.reshape(2, src.total()), dst2d, camera_matrix_, dist_coeffs_);

        // 2D object points + z-channel -> 3D object points
        dst.create(src.size(), CV_32FC3);
        // copy xy-channels
        dst2d.reshape(1, dst2d.total()).copyTo(dst.reshape(1, dst.total()).colRange(0, 2));
        // fill z-channel to 1.0
        dst.reshape(1, dst.total()).col(2).setTo(1.);

        mask.create(src.size(), CV_8UC1);
        const cv::Rect_<float> rect(cv::Point2f(0., 0.), src.size());
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                mask.at<unsigned char>(y, x) = rect.contains(src.at<cv::Point2f>(y, x)) ? 1 : 0;
            }
        }
    }

   private:
    cv::Matx33d camera_matrix_;
    std::vector<double> dist_coeffs_;
};
}

#endif /* _IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_PLUGIN_HPP_ */
