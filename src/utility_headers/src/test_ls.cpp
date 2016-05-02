#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <ros/init.h>
#include <utility_headers/ls.hpp>
#include <utility_headers/verbose_console.hpp>

namespace uhl = utility_headers::ls;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_ls");
    ros::NodeHandle handle;

    const std::vector<std::string> all(uhl::ls("*"));
    for (std::size_t i = 0; i < all.size(); ++i) {
        ROS_VINFO_STREAM(all[i]);
    }

    return 0;
}
