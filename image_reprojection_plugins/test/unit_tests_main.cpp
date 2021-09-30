#include <gtest/gtest.h>

#include <ros/init.h>
#include <ros/node_handle.h>

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "unit_tests");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}