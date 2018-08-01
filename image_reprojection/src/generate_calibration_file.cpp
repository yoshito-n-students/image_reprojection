#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <opencv2/core/core.hpp>

int main(int argc, char *argv[]) {
  const cv::CommandLineParser args(argc, argv,
                                   "{ help | | }"
                                   "{ @file-name | <none> | }"
                                   "{ camera-name | camera | }"
                                   "{ width | 0 | }"
                                   "{ height | 0 | }"
                                   "{ distortion-model | | }"
                                   "{ d-size | 5 | }");
  if (args.has("help")) {
    args.printMessage();
    return 0;
  }

  if (!args.check()) {
    args.printErrors();
    return 1;
  }

  sensor_msgs::CameraInfo camera_info;
  camera_info.width = args.get< int >("width");
  camera_info.height = args.get< int >("height");
  camera_info.distortion_model = args.has("distortion-model")
                                     ? args.get< std::string >("distortion-model")
                                     : sensor_msgs::distortion_models::PLUMB_BOB;
  camera_info.D.resize(args.get< int >("d-size"), 0.);

  camera_calibration_parsers::writeCalibration(args.get< std::string >("@file-name"),
                                               args.get< std::string >("camera-name"), camera_info);

  return 0;
}