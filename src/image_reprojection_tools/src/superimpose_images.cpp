#include <stdexcept>

#include <image_reprojection_tools/image_superimposition.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/spinner.h>
#include <ros/this_node.h>
#include <utility_headers/param.hpp>

int main(int argc, char* argv[]) {
    namespace irt = image_reprojection_tools;
    namespace uhp = utility_headers::param;

    ros::init(argc, argv, "superimpose_images", ros::init_options::AnonymousName);

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    try {
        irt::ImageSuperimposition node;
        node.init(ros::this_node::getName(), ros::names::getRemappings(), args);

        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
    } catch (const std::exception& ex) {
        ROS_ERROR_STREAM(ex.what());
    }

    return 0;
}