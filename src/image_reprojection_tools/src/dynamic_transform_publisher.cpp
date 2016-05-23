#include <ros/init.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <utility_headers/param.hpp>

namespace uhp = utility_headers::param;

int main(int argc, char *argv[]) {
    //
    // ROS initialization
    //

    ros::init(argc, argv, "dynamic_transform_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle handle;

    //
    // load parameters
    //

    tf::StampedTransform transform;
    transform.frame_id_ = uhp::param<std::string>("~frame_id", "world");
    transform.child_frame_id_ = uhp::param<std::string>("~child_frame_id", "robot");

    double x(uhp::param("~initial_x", 0.));
    double y(uhp::param("~initial_y", 0.));
    double z(uhp::param("~initial_z", 0.));
    double roll(uhp::param("~initial_roll", 0.));
    double pitch(uhp::param("~initial_pitch", 0.));
    double yaw(uhp::param("~initial_yaw", 0.));

    const double dx(uhp::param("~dx", 0.));
    const double dy(uhp::param("~dy", 0.));
    const double dz(uhp::param("~dz", 0.));
    const double droll(uhp::param("~droll", 0.));
    const double dpitch(uhp::param("~dpitch", 0.));
    const double dyaw(uhp::param("~dyaw", 0.));

    ros::Rate rate(uhp::param("~frequency", 10.));

    //
    // loop
    //

    tf::TransformBroadcaster broadcaster;

    while (ros::ok()) {
        // send transform
        {
            tf::Vector3 origin;
            origin.setValue(x, y, z);
            transform.setOrigin(origin);
        }
        {
            tf::Quaternion rotation;
            rotation.setRPY(roll, pitch, yaw);
            transform.setRotation(rotation);
        }
        transform.stamp_ = ros::Time::now() + rate.expectedCycleTime();
        broadcaster.sendTransform(transform);

        // increment transform
        x += dx;
        y += dy;
        z += dz;
        roll += droll;
        while (roll < -M_PI) {
            roll += 2 * M_PI;
        }
        while (roll > M_PI) {
            roll -= 2 * M_PI;
        }
        pitch += dpitch;
        while (pitch < -M_PI) {
            pitch += 2 * M_PI;
        }
        while (pitch > M_PI) {
            pitch -= 2 * M_PI;
        }
        yaw += dyaw;
        while (yaw < -M_PI) {
            yaw += 2 * M_PI;
        }
        while (yaw > M_PI) {
            yaw -= 2 * M_PI;
        }

        // sleep for a while
        rate.sleep();
    }

    return 0;
}
