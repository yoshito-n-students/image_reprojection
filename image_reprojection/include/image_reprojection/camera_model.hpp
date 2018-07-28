#ifndef IMAGE_REPROJECTION_CAMERA_MODEL_HPP
#define IMAGE_REPROJECTION_CAMERA_MODEL_HPP

#include <sensor_msgs/CameraInfo.h>

#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection {

class CameraModel {
public:
  CameraModel() {}

  virtual ~CameraModel() {}

  /*
    project 3D points in camera's optical coordinate to camera's 2D pixel coordinate.
    mask is input and output; as input it should indicate valid input points. 
    as output it should indicate successfully-projected points.
   */
  void project3dToPixel(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    /*
      required conditions
     */

    // source points must be an array of 3D points
    CV_Assert(src.type() == CV_32FC3);
    // mask must be an array of bytes
    CV_Assert(mask.type() == CV_8UC1);
    // sizes of source points and mask must be same
    CV_Assert(src.size() == mask.size());

    /*
      projection
     */

    onProject3dToPixel(src, dst, mask);

    /*
      ensured conditions
     */

    // projected points must be an array of 2D points
    CV_Assert(dst.type() == CV_32FC2);
    // mask must be an array of bytes
    CV_Assert(mask.type() == CV_8UC1);
    // sizes of source, projected points and mask must be same
    CV_Assert(src.size() == dst.size());
    CV_Assert(dst.size() == mask.size());
  }

  /*
    project points in camera's 2D pixel coordinate to rays in camera's 3D optical coordinate.
    mask is input and output; as input it should indicate valid input points. 
    as output it should indicate successfully-projected points.
   */
  void projectPixelTo3dRay(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const {
    /*
      required conditions
     */

    // source points must be an array of 2D points
    CV_Assert(src.type() == CV_32FC2);
    // mask must be an array of bytes
    CV_Assert(mask.type() == CV_8UC1);
    // sizes of source points and mask must be same
    CV_Assert(src.size() == mask.size());

    /*
      reprojection
     */

    onProjectPixelTo3dRay(src, dst, mask);

    /*
      ensured conditions
     */

    // reprojected points must be an array of 3D points
    CV_Assert(dst.type() == CV_32FC3);
    // mask must be an array of bytes
    CV_Assert(mask.type() == CV_8UC1);
    // sizes of source, reprojected points and mask must be same
    CV_Assert(src.size() == dst.size());
    CV_Assert(dst.size() == mask.size());
  }

  /*
    update model using the given camera info
  */
  virtual void fromCameraInfo(const sensor_msgs::CameraInfo &camera_info) = 0;

  /*
    export current model as camera info message
  */
  virtual sensor_msgs::CameraInfoPtr toCameraInfo() const = 0;

private:
  /*
    following vitual functions must be implemented in a child class
  */
  virtual void onProject3dToPixel(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const = 0;

  virtual void onProjectPixelTo3dRay(const cv::Mat &src, cv::Mat &dst, cv::Mat &mask) const = 0;
};

typedef boost::shared_ptr< CameraModel > CameraModelPtr;
typedef boost::shared_ptr< const CameraModel > CameraModelConstPtr;

} // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_CAMERA_MODEL_HPP */
