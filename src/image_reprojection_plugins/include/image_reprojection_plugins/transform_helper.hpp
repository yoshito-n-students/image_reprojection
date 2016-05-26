#ifndef IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP
#define IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP

#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>

namespace image_reprojection_plugins {

class TransformHelper {
   public:
    TransformHelper() { set(cv::Matx34f::eye()); }

    virtual ~TransformHelper() {}

    void set(const cv::Matx34f& m) {
        // set the transform matrix
        m_ = m;

        // set the inverse matrix
        cv::Matx44f m44(cv::Matx44f::eye());
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                m44(i, j) = m_(i, j);
            }
        }
        m_inv_ = m44.inv().get_minor<3, 4>(0, 0);
    }

    void set(const tf::Transform& tf_m) {
        cv::Matx34f cv_m;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cv_m(i, j) = tf_m.getBasis()[i][j];
            }
            cv_m(i, 3) = tf_m.getOrigin()[i];
        }
        set(cv_m);
    }

    cv::Matx34f get() const { return m_; }

    cv::Matx34f getInverse() const { return m_inv_; }

    void transform(const cv::Mat& src, cv::Mat& dst) const { cv::transform(src, dst, m_); }

    void inverseTransform(const cv::Mat& src, cv::Mat& dst) const { cv::transform(src, dst, m_inv_); }

   private:
    cv::Matx34f m_;
    cv::Matx34f m_inv_;
};
}
#endif  // IMAGE_REPROJECTION_PLUGINS_TRANSFORM_HELPER_HPP