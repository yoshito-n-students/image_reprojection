#ifndef IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP
#define IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class TransformHelper {
   public:
    TransformHelper() {
        set(cv::Matx44d::eye());
    }

    virtual ~TransformHelper() {}

    void set(const cv::Matx44d& transform) {
        transform_ = transform;
        inverse_transform_ = transform_.inv();
    }

    void transform(const cv::Mat& src, cv::Mat& dst) const { transform(src, transform_, dst); }

    void inverseTransform(const cv::Mat& src, cv::Mat& dst) const {
        transform(src, inverse_transform_, dst);
    }

   private:
    static void transform(const cv::Mat& src, const cv::Matx44d& m, cv::Mat& dst) {
        CV_Assert(src.type() == CV_32FC3);

        dst.create(src.size(), src.type());
        for (int x = 0; x < src.cols; ++x) {
            for (int y = 0; y < src.rows; ++y) {
                const cv::Vec3f & s(src.at<cv::Vec3f>(y,x));
                cv::Vec3f & d(dst.at<cv::Vec3f>(y,x));
                d[0] = m(0,0)*s[0] + m(0,1)*s[1] + m(0,2)*s[2] + m(0,3);
                d[1] = m(1,0)*s[0] + m(1,1)*s[1] + m(1,2)*s[2] + m(1,3);
                d[2] = m(2,0)*s[0] + m(2,1)*s[1] + m(2,2)*s[2] + m(2,3);
            }
        }
    }

   private:
    cv::Matx44d transform_;
    cv::Matx44d inverse_transform_;
};
}
#endif  // IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP