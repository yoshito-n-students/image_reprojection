#ifndef IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_HPP
#define IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_HPP

#include <cmath>
#include <vector>

#include <image_reprojection/projection_interface.hpp>
#include <image_reprojection_plugins/transform_helper.hpp>
#include <ros/console.h>
#include <utility_headers/param.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace image_reprojection_plugins {

class StandardProjection : public image_reprojection::ProjectionInterface {
   public:
    StandardProjection() {}

    virtual ~StandardProjection() {}

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        // get private node handle to access parameters
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        // load parameters
        uhp::getRequired(pnh, "camera_matrix", camera_matrix_);
        dist_coeffs_ = uhp::param(pnh, "dist_coeffs", std::vector<double>(4, 0.));
        const cv::Vec2d fov(uhp::param(pnh, "field_of_view", cv::Vec2d(M_PI / 2., M_PI / 2.)));
        frame_.width = std::tan(fov(0) / 2.) * 2.;
        frame_.height = std::tan(fov(1) / 2.) * 2.;
        frame_.x = -frame_.width / 2.;
        frame_.y = -frame_.height / 2.;
        transform_.set(uhp::param<cv::Matx34f>(pnh, "extrinsic_matrix", cv::Matx34f::eye()));

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        // transform source points into the camera coordinate
        cv::Mat cam;
        transform_.transform(src, cam);

        // project 3D points in the camera coordinate into the 2D image coordinate
        cv::projectPoints(cam.reshape(3, cam.total()), cv::Vec3d::all(0.), cv::Vec3d::all(0.),
                          camera_matrix_, dist_coeffs_, dst);
        dst = dst.reshape(2, cam.size().height);

        // mask points out of the projection frame
        mask.create(dst.size(), CV_8UC1);
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                const cv::Point3f& p(cam.at<cv::Point3f>(y, x));
                mask.at<unsigned char>(y, x) =
                    frame_.contains(cv::Point2f(p.x / p.z, p.y / p.z)) ? 1 : 0;
            }
        }
    }

    virtual void onReproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        // reproject 2D points in the image coordinate to the camera coordinate
        cv::Mat cam_2d;
        cv::undistortPoints(src.reshape(2, src.total()), cam_2d, camera_matrix_, dist_coeffs_);
        cam_2d = cam_2d.reshape(2, src.size().height);

        // add z-channel to the points in the camera coordinate
        cv::Mat cam(cam_2d.size(), CV_32FC3);
        // copy xy-channels
        cam_2d.reshape(1, cam_2d.total()).copyTo(cam.reshape(1, cam.total()).colRange(0, 2));
        // fill z-channel to 1.0
        cam.reshape(1, cam.total()).col(2).setTo(1.);

        // transform the points in the camera coordinate into the destination coordinate
        transform_.inverseTransform(cam, dst);

        // mask points out of the projection frame
        mask.create(dst.size(), CV_8UC1);
        for (int x = 0; x < mask.size().width; ++x) {
            for (int y = 0; y < mask.size().height; ++y) {
                const cv::Point3f& p(cam.at<cv::Point3f>(y, x));
                mask.at<unsigned char>(y, x) =
                    frame_.contains(cv::Point2f(p.x / p.z, p.y / p.z)) ? 1 : 0;
            }
        }
    }

   private:
    cv::Matx33d camera_matrix_;
    std::vector<double> dist_coeffs_;
    cv::Rect_<float> frame_;
    TransformHelper transform_;
};
}

#endif /* IMAGE_REPROJECTION_PLUGINS_STANDARD_PROJECTION_HPP */
