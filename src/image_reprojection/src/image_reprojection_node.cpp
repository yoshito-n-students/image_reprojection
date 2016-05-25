#include <image_reprojection/image_reprojection.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include <ros/this_node.h>
#include <utility_headers/param.hpp>

int main(int argc, char *argv[]) {
    namespace uhp = utility_headers::param;
    
    ros::init(argc, argv, "image_reprojection_node");
    ros::NodeHandle handle;

    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    image_reprojection::ImageReprojection node;
    node.init(ros::this_node::getName(), ros::names::getRemappings(), args);

    ROS_INFO_STREAM(ros::this_node::getName() << " has been started");

    ros::MultiThreadedSpinner spinner(uhp::param("~thread_count",2));
    spinner.spin();

    return 0;
}
