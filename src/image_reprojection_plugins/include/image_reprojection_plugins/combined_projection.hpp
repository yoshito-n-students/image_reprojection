#ifndef IMAGE_REPROJECTION_PLUGINS_COMBINED_PROJECTION_HPP
#define IMAGE_REPROJECTION_PLUGINS_COMBINED_PROJECTION_HPP

#include <string>
#include <vector>

#include <image_reprojection/projection_interface.hpp>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <utility_headers/param.hpp>

#include <opencv2/core/core.hpp>

#include <XmlRpcValue.h>

namespace image_reprojection_plugins {

class CombinedProjection : public image_reprojection::ProjectionInterface {
   private:
    typedef image_reprojection::ProjectionInterface Projection;
    typedef image_reprojection::ProjectionInterfacePtr ProjectionPtr;
    typedef std::vector<ProjectionPtr> ProjectionPtrs;

   public:
    CombinedProjection()
        : loader_("image_reprojection", "image_reprojection::ProjectionInterface") {}

    virtual ~CombinedProjection() {
        // destroy all projections loaded from the plugin loader
        // prior to the destruction of the loader
        projections_.clear();
    }

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        // get private node handle to access parameters
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        // load projections, each type is a value of "~/*/type"
        {
            // get the parameter tree for this nodelet
            typedef XmlRpc::XmlRpcValue Xrv;
            const Xrv params(uhp::param(pnh, "", Xrv()));
            if (params.getType() == Xrv::TypeStruct) {
                // iterate "~/*"
                for (Xrv::ValueStruct::const_iterator child = const_cast<Xrv&>(params).begin();
                     child != const_cast<Xrv&>(params).end(); ++child) {
                    // try to get "~/*/type"
                    std::string type;
                    if (!uhp::get(pnh, ros::names::append(child->first, "type"), type)) {
                        continue;
                    }
                    // load and init a plugin whose name is the value of "~/*/type"
                    const ProjectionPtr projection(loader_.createInstance(type));
                    projection->init(pnh.resolveName(child->first), ros::M_string(), getMyArgv());
                    projections_.push_back(projection);
                }
            }
        }

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onProject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        // init the 2D destination points
        dst.create(src.size(), CV_32FC2);
        mask = cv::Mat::zeros(src.size(), CV_8UC1);

        // do all projections
        for (ProjectionPtrs::iterator projection = projections_.begin();
             projection != projections_.end(); ++projection) {
            // project the source points
            cv::Mat this_dst, this_mask;
            (*projection)->project(src, this_dst, this_mask);

            // update the destination points and mask
            this_dst.copyTo(dst, this_mask);
            mask = cv::max(mask, this_mask);
        }
    }

    virtual void onReproject(const cv::Mat& src, cv::Mat& dst, cv::Mat& mask) {
        // init the 3D destination points
        dst.create(src.size(), CV_32FC3);
        mask = cv::Mat::zeros(src.size(), CV_8UC1);

        // do all reprojections in the reverse order
        for (ProjectionPtrs::iterator projection = projections_.begin();
             projection != projections_.end(); ++projection) {
            // reproject the source points
            cv::Mat this_dst, this_mask;
            (*projection)->reproject(src, this_dst, this_mask);

            // update the destination points and mask
            this_dst.copyTo(dst, this_mask);
            mask = cv::max(mask, this_mask);
        }
    }

   private:
    pluginlib::ClassLoader<Projection> loader_;
    ProjectionPtrs projections_;
};
}

#endif  // IMAGE_REPROJECTION_PLUGINS_COMBINED_PROJECTION_HPP