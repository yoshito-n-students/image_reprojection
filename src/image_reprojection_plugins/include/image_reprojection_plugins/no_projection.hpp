#ifndef IMAGE_REPROJECTION_PLUGINS_NO_PROJECTION_HPP
#define IMAGE_REPROJECTION_PLUGINS_NO_PROJECTION_HPP

#include <image_reprojection/projection_interface.hpp>
#include <ros/console.h>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class NoProjection : public image_reprojection::ProjectionInterface {
   public:
    NoProjection() {}

    virtual ~NoProjection() {}

   private:
    virtual void onInit() { ROS_INFO_STREAM(getName() << " has been initialized"); }

    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        dst.create(src.size(), CV_32FC2);
        // dst(i,j).x = src(i,j).x / src(i,j).z
        cv::divide(src.reshape(1, src.total()).col(0), src.reshape(1, src.total()).col(2),
                   dst.reshape(1, dst.total()).col(0));
        // dst(i,j).y = src(i,j).y / src(i,j).z
        cv::divide(src.reshape(1, src.total()).col(1), src.reshape(1, src.total()).col(2),
                   dst.reshape(1, dst.total()).col(1));

        mask = cv::Mat::ones(src.size(), CV_8UC1);
    }

    virtual void onReproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        dst.create(src.size(), CV_32FC3);
        // dst(i,j).x = src(i,j).x
        // dst(i,j).y = src(i,j).y
        dst.reshape(1, dst.total()).colRange(0, 2) = src.reshape(1, src.total());
        // dst(i,j).z = 1.
        dst.reshape(1, dst.total()).col(2) = 1.;

        mask = cv::Mat::ones(src.size(), CV_8UC1);
    }
};
}

#endif /* IMAGE_REPROJECTION_PLUGINS_NO_PROJECTION_HPP */
