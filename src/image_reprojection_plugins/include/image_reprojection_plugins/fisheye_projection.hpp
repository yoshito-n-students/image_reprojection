#ifndef IMAGE_REPROJECTION_PLUGINS_FISHEYE_PROJECTION_HPP
#define IMAGE_REPROJECTION_PLUGINS_FISHEYE_PROJECTION_HPP

#include <cmath>
#include <vector>

#include <cv_extension/calib3d/calib3d.hpp>
#include <image_reprojection/projection_interface.hpp>
#include <ros/console.h>
#include <utility_headers/param.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class FisheyeProjection : public image_reprojection::ProjectionInterface {
   public:
    FisheyeProjection() {}

    virtual ~FisheyeProjection() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        // get private node handle to access parameters
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        // load parameters
        uhp::getRequired(pnh, "camera_matrix", camera_matrix_);
        dist_coeffs_ = uhp::param(pnh, "dist_coeffs", std::vector<double>(4, 0.));
        min_z_ = std::cos(uhp::param<double>(pnh, "field_of_view", M_PI) / 2.);

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        cv::fisheye::projectPoints(src.reshape(3, src.total()), dst, cv::Vec3d::all(0.),
                                   cv::Vec3d::all(0.), camera_matrix_, dist_coeffs_);
        dst = dst.reshape(2, src.size().height);

        mask.create(src.size(), CV_8UC1);
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                const cv::Point3f& p(src.at<cv::Point3f>(y, x));
                mask.at<unsigned char>(y, x) = (p.z / cv::norm(p) > min_z_) ? 1 : 0;
            }
        }
    }

    virtual void onReproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        cv::fisheye::undistortPointsTo3D(src.reshape(2, src.total()), dst, camera_matrix_,
                                         dist_coeffs_);
        dst = dst.reshape(3, src.size().height);

        mask.create(src.size(), CV_8UC1);
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                const cv::Point3f& p(dst.at<cv::Point3f>(y, x));
                mask.at<unsigned char>(y, x) = (p.z / cv::norm(p) > min_z_) ? 1 : 0;
            }
        }
    }

   private:
    // parameters
    cv::Matx33d camera_matrix_;
    std::vector<double> dist_coeffs_;
    double min_z_;
};
}

#endif /* IMAGE_REPROJECTION_PLUGINS_FISHEYE_PROJECTION_HPP */
