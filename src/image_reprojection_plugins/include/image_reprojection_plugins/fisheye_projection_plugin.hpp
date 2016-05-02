#ifndef _IMAGE_REPROJECTION_PLUGINS_FISHEYE_PROJECTION_PLUGIN_HPP_
#define _IMAGE_REPROJECTION_PLUGINS_FISHEYE_PROJECTION_PLUGIN_HPP_

#include <boost/array.hpp>

#include <opencv2/core/core.hpp>

#include <cv_extension/calib3d/calib3d.hpp>
#include <image_reprojection/projection_plugin.hpp>
#include <ros/console.h>
#include <utility_headers/param.hpp>

namespace image_reprojection_plugins {

class FisheyeProjectionPlugin : public image_reprojection::ProjectionPlugin {
   public:
    FisheyeProjectionPlugin() {}

    virtual ~FisheyeProjectionPlugin() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        {
            boost::array<boost::array<double, 3>, 3> mat;
            uhp::getRequired(pnh, "camera_matrix", mat);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    camera_matrix_(i, j) = mat[i][j];
                }
            }
        }

        uhp::getRequired(pnh, "dist_coeffs", dist_coeffs_);

        {
            const double fov(uhp::param(pnh, "field_of_view", M_PI));
            min_z_ = std::cos(fov / 2.);
        }

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

        mask = cv::Mat::ones(src.size(), CV_8UC1);
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                const cv::Point3f& p(dst.at<cv::Point3f>(y, x));
                mask.at<unsigned char>(y, x) = (p.z / cv::norm(p) > min_z_) ? 1 : 0;
            }
        }
    }

   private:
    cv::Matx33d camera_matrix_;
    std::vector<double> dist_coeffs_;
    double min_z_;
};
}

#endif /* _IMAGE_REPROJECTION_PLUGINS_FISHEYE_PROJECTION_PLUGIN_HPP_ */
