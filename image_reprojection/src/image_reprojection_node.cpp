#include <image_reprojection/image_reprojection.hpp>
#include <param_utilities/param_utilities.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <ros/this_node.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "image_reprojection_node");
    ros::NodeHandle handle;

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    image_reprojection::ImageReprojection node;
    node.init(ros::this_node::getName(), ros::names::getRemappings(), args);

    ROS_INFO_STREAM(ros::this_node::getName() << " has been started");

    ros::MultiThreadedSpinner spinner(param_utilities::param("~thread_count", 2));
    spinner.spin();

    return 0;
}
