#ifndef IMAGE_REPROJECTION_PLUGINS_TEST_RANDOM_HPP
#define IMAGE_REPROJECTION_PLUGINS_TEST_RANDOM_HPP

#include <geometry_msgs/Point.h>

#include <opencv2/core/core.hpp>

// the global random number generator
static cv::RNG g_rng(std::time(NULL));

// generate random value between a and b
static inline double randomValue(const double a, const double b) { return g_rng.uniform(a, b); }

// generate random value between a and b, except zero
static inline double randomNonZeroValue(const double a, const double b) {
  while (true) {
    const double value(randomValue(a, b));
    if (value != 0.) {
      return value;
    }
  }
}

// generate random pixel coordinate in given region
static inline cv::Vec2f randomPixel(const cv::Rect_<float> &region) {
  return cv::Vec2f(randomValue(region.x, region.x + region.width),
                   randomValue(region.y, region.y + region.height));
}

// generate random point, each element is between a and b
static inline cv::Vec3f randomPoint(const double a, const double b) {
  return cv::Vec3f(randomValue(a, b), randomValue(a, b), randomValue(a, b));
}

// generate random point whose norm is not zero, each element is between a and b
static inline cv::Vec3f randomNonZeroPoint(const double a, const double b) {
  while (true) {
    const cv::Vec3f point(randomPoint(a, b));
    for (int i = 0; i < 3; ++i) {
      if (point[i] != 0) {
        return point;
      }
    }
  }
}

// generate set of pixels by using randomPixel()
static inline cv::Mat randomPixels(const cv::Size &size, const cv::Rect_<float> &region) {
  cv::Mat pixels(size, CV_32FC2);
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      pixels.at<cv::Vec2f>(y, x) = randomPixel(region);
    }
  }
  return pixels;
}

// generate set of points by using randomPoint()
static inline cv::Mat randomPoints(const cv::Size &size, const double a, const double b) {
  cv::Mat points(size, CV_32FC3);
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      points.at<cv::Vec3f>(y, x) = randomPoint(a, b);
    }
  }
  return points;
}

// generate set of points by using randomNonZeroPoint()
static inline cv::Mat randomNonZeroPoints(const cv::Size &size, const double a, const double b) {
  cv::Mat points(size, CV_32FC3);
  for (int x = 0; x < size.width; ++x) {
    for (int y = 0; y < size.height; ++y) {
      points.at<cv::Vec3f>(y, x) = randomNonZeroPoint(a, b);
    }
  }
  return points;
}

// cv::Vec3f to geometry_msgs::Point
static inline geometry_msgs::Point toPointMsg(const cv::Vec3f &point) {
  geometry_msgs::Point msg;
  msg.x = point[0];
  msg.y = point[1];
  msg.z = point[2];
  return msg;
}

#endif