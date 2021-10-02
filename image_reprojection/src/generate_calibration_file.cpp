#include <iostream>
#include <string>

#include <camera_calibration_parsers/parse.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

#include <opencv2/core/core.hpp>

int main(int argc, char *argv[]) {
  const cv::CommandLineParser args(argc, argv,
                                   "{ help | | show help message }"
                                   "{ @file-name | <none> | output calibration file }"
                                   "{ @camera-name | <none> | camera name }"
                                   "{ width | | image width when calibrated }"
                                   "{ height | | image height when calibrated }"
                                   "{ distortion-model | | distortion model }"
                                   "{ d-size | | number of distortion parameters }"
                                   "{ fx | | focal length x }"
                                   "{ fy | | focal length y }"
                                   "{ cx | | principal point x }"
                                   "{ cy | | principal point y }");

  // show help and exit as needed
  if (args.has("help")) {
    args.printMessage();
    return 0;
  }

  // access required arguments to check errors
  const std::string file_name = args.get<std::string>("@file-name");
  const std::string camera_name = args.get<std::string>("@camera-name");
  if (!args.check()) {
    args.printErrors();
    return 1;
  }

  // fill camera calibration data according to arguments
  sensor_msgs::CameraInfo camera_info;
  // image size
  camera_info.width = args.has("width") ? args.get<int>("width") : 0;
  camera_info.height = args.has("height") ? args.get<int>("height") : 0;
  // distortion
  camera_info.distortion_model = args.has("distortion-model")
                                     ? args.get<std::string>("distortion-model")
                                     : sensor_msgs::distortion_models::PLUMB_BOB;
  camera_info.D.resize(args.has("d-size") ? args.get<int>("d-size") : 5, 0.);
  // intrinsic matrix
  const double fx = args.has("fx") ? args.get<double>("fx") : 1.,
               cx = args.has("cx") ? args.get<double>("cx") : (camera_info.width / 2.),
               fy = args.has("fy") ? args.get<double>("fy") : 1.,
               cy = args.has("cy") ? args.get<double>("cy") : (camera_info.height / 2.);
  camera_info.K = {fx, 0., cx, //
                   0., fy, cy, //
                   0., 0., 1.};
  // rotation matrix (meaningless for monocular camera)
  camera_info.R = {1., 0., 0., //
                   0., 1., 0., //
                   0., 0., 1.};
  // projection matrix ([K | 0] for monocular camera)
  camera_info.P = {camera_info.K[0], camera_info.K[1], camera_info.K[2], 0., //
                   camera_info.K[3], camera_info.K[4], camera_info.K[5], 0., //
                   camera_info.K[6], camera_info.K[7], camera_info.K[8], 0.};

  // write calibration file
  if (camera_calibration_parsers::writeCalibration(file_name, camera_name, camera_info)) {
    std::cout << "Wrote camera calibration to " << file_name << std::endl;
  } else {
    std::cerr << "Failed to write camera calibration to " << file_name << std::endl;
  }

  return 0;
}