#include <ros/console.h>
#include <ros/init.h>

#include <utility_headers/verbose_console.hpp>

#include <gtest/gtest.h>

namespace rcl = ros::console::levels;
namespace uhc = utility_headers::console;

TEST(TestVerboseConsole,CasePrintf){
  uhc::VerboseFilter::showNode(rcl::Debug);
  uhc::VerboseFilter::showFunction(rcl::Warn);
  uhc::VerboseFilter::showFile(rcl::Error);

  ROS_VDEBUG("debug message");
  ROS_VINFO("info message");
  ROS_VWARN("warn message");
  ROS_VERROR("error message");
  ROS_VFATAL("fatal message");
}

TEST(TestVerboseConsole,CaseStream){
  uhc::VerboseFilter::showNode(rcl::Fatal);
  uhc::VerboseFilter::showFunction(rcl::Info);
  uhc::VerboseFilter::showFile(rcl::Debug);

  ROS_VDEBUG_STREAM("debug message via stream");
  ROS_VINFO_STREAM("info message via stream");
  ROS_VWARN_STREAM("warn message via stream");
  ROS_VERROR_STREAM("error message via stream");
  ROS_VFATAL_STREAM("fatal message via stream");
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc,argv);

  ros::init(argc,argv,"test_verbose_console");

  return RUN_ALL_TESTS();
}
