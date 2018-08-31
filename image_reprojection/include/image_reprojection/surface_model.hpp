#ifndef IMAGE_REPROJECTION_SURFACE_MODEL_HPP
#define IMAGE_REPROJECTION_SURFACE_MODEL_HPP

#include <string>

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>

#include <boost/shared_ptr.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection {

/*
  base class of surface models which calculate intersections vs 3D rays.
  child class may throw std::exception from its overrode functions.
  also, child class must be threadsafe.
*/
class SurfaceModel : public nodelet::Nodelet {
public:
  SurfaceModel() {}

  virtual ~SurfaceModel() {}

  /*
    calculate intersection between 3D ray in surface coordinate and surface.
    input ray directions have not to be normalized.
    mask is input and output; as input it should indicate valid input rays.
    as output it should indicate rays which intersects the surface.
  */
  void intersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction, cv::Mat &dst,
                    cv::Mat &mask) const {
    /*
      required conditions
    */

    // source points must be an array of 3D points
    CV_Assert(src_direction.type() == CV_32FC3);
    // mask must be an array of bytes
    CV_Assert(mask.type() == CV_8UC1);
    // sizes of source points and mask must be same
    CV_Assert(src_direction.size() == mask.size());

    /*
      intersection calculation
    */

    onIntersection(src_origin, src_direction, dst, mask);

    /*
      ensured conditions
    */

    // intersection points must be an array of 3D points
    CV_Assert(dst.type() == CV_32FC3);
    // mask must be an array of bytes
    CV_Assert(mask.type() == CV_8UC1);
    // sizes of source, intersection points and mask must be same
    CV_Assert(src_direction.size() == dst.size());
    CV_Assert(dst.size() == mask.size());
  }

  /*
     checks 3D rays from ray origin and surface intersect at expected points within errors.
     ray origin and expected intersection points are assumed to be described in surface coordinate.
     mask is input and output; as input it should indicate valid input intersections.
     as output it should indicate intersections which satisfy error conditions.
   */
  void intersectsAt(const cv::Vec3f &src_origin, const cv::Mat &expected, cv::Mat &mask,
                    const double abs_error) const {
    /*
      required conditions
    */

    // allowable error must be zero or positive
    CV_Assert(abs_error >= 0.);

    /*
      calculation of actual intersection
    */

    cv::Mat actual;
    intersection(src_origin, expected - cv::Mat(expected.size(), CV_32FC3).setTo(src_origin),
                 actual, mask);

    /*
      update mask according to error on intersection points
     */

    for (int x = 0; x < mask.size().width; ++x) {
      for (int y = 0; y < mask.size().height; ++y) {
        unsigned char &m(mask.at< unsigned char >(y, x));
        const cv::Vec3f &a(actual.at< cv::Vec3f >(y, x));
        const cv::Vec3f &e(expected.at< cv::Vec3f >(y, x));
        m = (m != 0 && cv::norm(a, e) <= abs_error) ? 1 : 0;
      }
    }
  }

  /*
    update this model
  */
  virtual void update(const topic_tools::ShapeShifter &surface) = 0;

  /*
    return surface frame id
  */
  virtual std::string getFrameId() const = 0;

private:
  /*
    following vitual functions must be implemented in a child class
  */
  virtual void onInit() = 0; // from nodelet::Nodelet

  virtual void onIntersection(const cv::Vec3f &src_origin, const cv::Mat &src_direction,
                              cv::Mat &dst, cv::Mat &mask) const = 0;
};

typedef boost::shared_ptr< SurfaceModel > SurfaceModelPtr;
typedef boost::shared_ptr< const SurfaceModel > SurfaceModelConstPtr;

} // namespace image_reprojection

#endif /* IMAGE_REPROJECTION_SURFACE_MODEL_HPP */
