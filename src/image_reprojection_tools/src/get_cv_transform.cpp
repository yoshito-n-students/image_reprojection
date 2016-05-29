#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ros/console.h>
#include <ros/init.h>
#include <utility_headers/param.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char* argv[]) {
    namespace uhp = utility_headers::param;

    ros::init(argc, argv, "get_cv_transform", ros::init_options::AnonymousName);

    try {
        // load source and destination points
        std::vector<cv::Vec2f> src;
        uhp::getRequired("~src", src);
        std::vector<cv::Vec2f> dst;
        uhp::getRequired("~dst", dst);
        CV_Assert((src.size() == 3 || src.size() == 4) && (src.size() == dst.size()));

        // stringize given points
        std::ostringstream oss;
        oss << "\n"
            << "Given:\n"
            << "  src: ";
        uhp::Helper<std::vector<cv::Vec2f> >::write(src, oss);
        oss << "\n"
            << "  dst: ";
        uhp::Helper<std::vector<cv::Vec2f> >::write(dst, oss);
        oss << "\n";

        // get a transform
        oss << "Result:\n";
        if (src.size() == 3) {
            // affine transform
            oss << "  affine transform: ";
            uhp::Helper<cv::Matx23f>::write(cv::getAffineTransform(src, dst), oss);
        } else {
            // perspective transform
            oss << "  perspective transform: ";
            uhp::Helper<cv::Matx33f>::write(cv::getPerspectiveTransform(src, dst), oss);
        }
        oss << "\n";

        // print a report in yaml format
        {
            std::string out(oss.str());
            std::replace(out.begin(), out.end(), '{', '[');
            std::replace(out.begin(), out.end(), '}', ']');
            ROS_INFO_STREAM(out);
        }
    } catch (const std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    return 0;
}