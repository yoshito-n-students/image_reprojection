#ifndef IMAGE_REPROJECTION_PROJECTION_INTERFACE_HPP
#define IMAGE_REPROJECTION_PROJECTION_INTERFACE_HPP

#include <nodelet/nodelet.h>

#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection {

class ProjectionInterface : public nodelet::Nodelet {
   public:
    ProjectionInterface() {}

    virtual ~ProjectionInterface() {}

    /*
      project points in the device 3D coordinate to the image 2D coordinate.
      mask will indicate successfully-projected elements
     */
    void project(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        /*
          required conditions
         */

        // source points must be an array of 3D points
        CV_Assert(src.type() == CV_32FC3);

        /*
          projection
         */

        onProject(src, dst, mask);

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
      reproject points in the image 2D coordinate to the device 3D coordinate.
      mask will indicate successfully-reprojected elements
     */
    void reproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        /*
          required conditions
         */

        // source points must be an array of 2D points
        CV_Assert(src.type() == CV_32FC2);

        /*
          reprojection
         */

        onReproject(src, dst, mask);

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

   private:
    /*
      below vitual functions must be implemented in a child class
      as well as void onInit() from nodelet::Nodelet
    */
    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) = 0;

    virtual void onReproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) = 0;
};

typedef boost::shared_ptr<ProjectionInterface> ProjectionInterfacePtr;
typedef boost::shared_ptr<const ProjectionInterface> ProjectionInterfaceConstPtr;
}

#endif /* IMAGE_REPROJECTION_PROJECTION_INTERFACE_HPP */
