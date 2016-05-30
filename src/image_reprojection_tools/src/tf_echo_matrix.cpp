#include <algorithm>
#include <sstream>
#include <string>

#include <image_reprojection_plugins/transform_helper.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <tf/transform_listener.h>
#include <utility_headers/param.hpp>

int main(int argc, char *argv[]) {
    namespace irp = image_reprojection_plugins;
    namespace uhp = utility_headers::param;

    ros::init(argc, argv, "tf_echo_matrix", ros::init_options::AnonymousName);

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    if (args.size() != 3) {
        ROS_ERROR("tf_echo_matrix <source_frame> <target_frame>");
        return 0;
    }

    // load parameters
    ros::NodeHandle pnh("~");
    ros::Rate rate(pnh.param<double>("rate", 1.));

    // wait for transform available
    tf::TransformListener listener;
    listener.waitForTransform(args[2], args[1], ros::Time(0), ros::Duration(1.));

    // periodically print the transform
    irp::TransformHelper helper;
    while (ros::ok()) {
        try {
            // try to get the latest transform
            tf::StampedTransform transform;
            listener.lookupTransform(args[2], args[1], ros::Time(0), transform);

            // print the latest transform matrix in yaml format
            helper.set(transform);
            {
                std::ostringstream oss;
                uhp::Helper<cv::Matx34f>::write(helper.get(), oss);
                std::string str(oss.str());
                std::replace(str.begin(), str.end(), '{', '[');
                std::replace(str.begin(), str.end(), '}', ']');
                ROS_INFO_STREAM(args[1] << " -> " << args[2] << " : " << str);
            }
        } catch (const tf::TransformException &ex) {
            ROS_ERROR_STREAM(ex.what());
            continue;
        }

        // sleep for a while
        rate.sleep();
    }

    return 0;
}