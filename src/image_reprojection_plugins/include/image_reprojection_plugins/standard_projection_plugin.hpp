#ifndef _IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_PLUGIN_HPP_
#define _IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_PLUGIN_HPP_

#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <image_reprojection/projection_plugin.hpp>
#include <ros/console.h>

namespace image_reprojection_plugins {

class StandardProjectionPlugin : public image_reprojection::ProjectionPlugin {
   public:
    StandardProjectionPlugin() {}

    virtual ~StandardProjectionPlugin() {}

   private:
    virtual void onInit() { ROS_DEBUG_STREAM(getName() << " has been initialized"); }

    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        cv::projectPoints(src.reshape(1, src.total()), cv::Vec3d::all(0.), cv::Vec3d::all(0.),
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
        cv::Mat tmp;
        cv::undistortPoints(src.reshape(1, src.total()), tmp, camera_matrix_, dist_coeffs_);
        dst.create(src.size(), CV_32FC3);
        dst.reshape(1, dst.total()).colRange(0, 2) = tmp;
        dst.reshape(1, dst.total()).col(2) = 1.;

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
