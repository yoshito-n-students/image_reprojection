#ifndef IMAGE_REPROJECTION_PLUGINS_COMBINED_TRANSFORM_HPP
#define IMAGE_REPROJECTION_PLUGINS_COMBINED_TRANSFORM_HPP

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

class CombinedTransform : public image_reprojection::TransformInterface {
   private:
    typedef image_reprojection::TransformInterface Transform;
    typedef image_reprojection::TransformInterfacePtr TransformPtr;
    typedef std::vector<TransformPtr> TransformPtrs;

   public:
    CombinedTransform() : loader_("image_reprojection", "image_reprojection::TransformInterface") {}

    virtual ~CombinedTransform() {
        // destroy all transforms loaded from the plugin loader
        // prior to the destruction of the loader
        transforms_.clear();
    }

   private:
    virtual void onInit() {
        namespace uhp = utility_headers::param;

        // get private node handle to access parameters
        const ros::NodeHandle& pnh(getPrivateNodeHandle());

        // load transforms, each type is a value of "~/*/type"
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
                    const TransformPtr transform(loader_.createInstance(type));
                    transform->init(pnh.resolveName(child->first), ros::M_string(), getMyArgv());
                    transforms_.push_back(transform);
                }
            }
        }

        ROS_INFO_STREAM(getName() << " has been initialized");
    }

    virtual void onTransform(const cv::Mat& src, cv::Mat& dst) {
        // chain all transfroms
        cv::Mat dst_prev(src);
        for (TransformPtrs::iterator transform = transforms_.begin();
             transform != transforms_.end(); ++transform) {
            (*transform)->transform(dst_prev, dst);
            dst_prev = dst;
        }
    }

    virtual void onInverseTransform(const cv::Mat& src, cv::Mat& dst) {
        // chain all transfroms
        cv::Mat dst_prev(src);
        for (TransformPtrs::iterator transform = transforms_.begin();
             transform != transforms_.end(); ++transform) {
            (*transform)->inverseTransform(dst_prev, dst);
            dst_prev = dst;
        }
    }

   private:
    pluginlib::ClassLoader<Transform> loader_;
    TransformPtrs transforms_;
};
}

#endif  // IMAGE_REPROJECTION_PLUGINS_COMBINED_TRANSFORM_HPP