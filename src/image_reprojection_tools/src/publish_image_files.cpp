#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/param.h>
#include <ros/rate.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[]) {
    //
    // ROS initialization
    //

    ros::init(argc, argv, "publish_image_files", ros::init_options::AnonymousName);
    ros::NodeHandle handle;

    //
    // load parameters
    //

    std::string frame_id;
    ros::param::param<std::string>("~frame_id", frame_id, "camera");

    std::string encoding;
    ros::param::param<std::string>("~encoding", encoding, "bgr8");

    double frequency;
    ros::param::param("~frequency", frequency, 10.);

    std::string image_path;
    if (!ros::param::get("~image_path", image_path)) {
        ROS_ERROR("~image_path must be given");
        return 0;
    }

    //
    // load image files
    //

    std::vector<sensor_msgs::ImagePtr> image_msgs;
    if (boost::filesystem::is_directory(image_path)) {
        boost::filesystem::recursive_directory_iterator entry(image_path);
        boost::filesystem::recursive_directory_iterator entry_end;
        for (; entry != entry_end; ++entry) {
            cv_bridge::CvImage image;
            image.header.frame_id = frame_id;
            image.encoding = encoding;
            image.image = cv::imread(entry->path().native());
            if (!image.image.empty()) {
                image_msgs.push_back(image.toImageMsg());
            }
        }
    } else {
        cv_bridge::CvImage image;
        image.header.frame_id = frame_id;
        image.encoding = encoding;
        image.image = cv::imread(image_path);
        if (!image.image.empty()) {
            image_msgs.push_back(image.toImageMsg());
        }
    }
    if (image_msgs.empty()) {
        ROS_ERROR_STREAM("No image was loaded from " << image_path);
        return 0;
    }

    //
    // launch image publisher
    //

    image_transport::ImageTransport image_handle(handle);
    image_transport::Publisher image_publisher(image_handle.advertise("image", 1));

    //
    // main loop
    //

    ros::Rate rate(frequency);
    for (unsigned int seq = 0; ros::ok(); ++seq) {
        sensor_msgs::ImagePtr msg(image_msgs[seq % image_msgs.size()]);
        msg->header.seq = seq;
        msg->header.stamp = ros::Time::now();
        image_publisher.publish(msg);
        rate.sleep();
    }

    return 0;
}
