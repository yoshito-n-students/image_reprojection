#ifndef IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP
#define IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP

#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class TransformHelper {
   public:
    TransformHelper() { set(cv::Matx34f::eye()); }

    virtual ~TransformHelper() {}

    void set(const cv::Matx34f& transform) {
        // v1 = R * v2 + t
        // -> v2 = inv(R) * (v1 - t)
        // -> v2 = inv(R) * v1 - inv(R) * t
        R_ = transform.get_minor<3, 3>(0, 0);
        t_ = transform.get_minor<3, 1>(0, 3);
        R_inv_ = R_.inv();
        t_inv_ = -R_inv_ * t_;
    }

    void set(const tf::Transform& transform) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_(i, j) = transform.getBasis()[i][j];
            }
            t_(i) = transform.getOrigin()[i];
        }
        R_inv_ = R_.inv();
        t_inv_ = -R_inv_ * t_;
    }

    cv::Matx34f get() const {
        cv::Matx34f transform;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                transform(i, j) = R_(i, j);
            }
            transform(i, 3) = t_(i);
        }
        return transform;
    }

    void transform(const cv::Mat& src, cv::Mat& dst) const { transform(src, R_, t_, dst); }

    void inverseTransform(const cv::Mat& src, cv::Mat& dst) const {
        transform(src, R_inv_, t_inv_, dst);
    }

   private:
    static void transform(const cv::Mat& src, const cv::Matx33f& R, const cv::Matx31f& t,
                          cv::Mat& dst) {
        CV_Assert(src.type() == CV_32FC3);

        dst.create(src.size(), src.type());
        for (int x = 0; x < src.cols; ++x) {
            for (int y = 0; y < src.rows; ++y) {
                dst.at<cv::Matx31f>(y, x) = R * src.at<cv::Matx31f>(y, x) + t;
            }
        }
    }

   private:
    cv::Matx33f R_;
    cv::Matx31f t_;
    cv::Matx33f R_inv_;
    cv::Matx31f t_inv_;
};
}
#endif  // IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP