#ifndef _IMAGE_REPROJECTION_TRANSFORM_PLUGIN_HPP_
#define _IMAGE_REPROJECTION_TRANSFORM_PLUGIN_HPP_

#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>

#include <nodelet/nodelet.h>

namespace image_reprojection{

  class TransformPlugin:
    public nodelet::Nodelet{
  public:
    TransformPlugin(){
    }

    virtual ~TransformPlugin(){
    }

    /*
      transform points in the device 3D coordinate.
    */
    void transform(const cv::Mat & src,cv::Mat & dst){
      /*
	required conditions
       */

      // source points must be an array of 3D points
      CV_Assert(src.type() == CV_32FC3);

      /*
	transform
       */

      onTransform(src,dst);

      /*
	ensured conditions
       */

      // transformed points must be an array of 3D points
      CV_Assert(dst.type() == CV_32FC3);
    }

    /*
      invert points in the device 3D coordinate.
    */
    void inverseTransform(const cv::Mat & src,cv::Mat & dst){
      /*
	required conditions
       */

      // source points must be an array of 3D points
      CV_Assert(src.type() == CV_32FC3);

      /*
	inverse transform
       */

      onInverseTransform(src,dst);

      /*
	ensured conditions
       */

      // inverted points must be an array of 3D points
      CV_Assert(dst.type() == CV_32FC3);
    }

  private:
    /*
      below vitual functions must be implemented in a child class
      as well as void onInit() from nodelet::Nodelet
    */
    virtual void onTransform(const cv::Mat & src,cv::Mat & dst) = 0;

    virtual void onInverseTransform(const cv::Mat & src,cv::Mat & dst) = 0;
  };

  typedef boost::shared_ptr<TransformPlugin> TransformPluginPtr;
  typedef boost::shared_ptr<const TransformPlugin> TransformPluginConstPtr;

}

#endif /* _IMAGE_REPROJECTION_TRANSFORM_PLUGIN_HPP_ */
