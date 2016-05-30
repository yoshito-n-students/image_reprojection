#ifndef IMAGE_REPROJECTION_TOOLS_IMAGE_SUPERIMPOSITION_PARAMS_HPP
#define IMAGE_REPROJECTION_TOOLS_IMAGE_SUPERIMPOSITION_PARAMS_HPP

#include <string>
#include <vector>

#include <utility_headers/param.hpp>

#include <opencv2/core/core.hpp>

namespace image_reprojection_tools {

struct ImageSuperimpositionParams {
    std::string topic;
    std::string transport;
    std::vector<cv::Vec2f> aoi;        // area of interest
    std::vector<cv::Vec3f> transform;  // 2x3 (affine) or 3x3 (perspective) transform matrix
    double transparency;
    bool primary;

    ImageSuperimpositionParams()
        : topic("image"), transport("raw"), transparency(0.), primary(false) {}
};
}

namespace utility_headers {
namespace param {

template <>
struct Helper<image_reprojection_tools::ImageSuperimpositionParams> {
    typedef Helper<image_reprojection_tools::ImageSuperimpositionParams> ThisType;
    typedef image_reprojection_tools::ImageSuperimpositionParams ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        if (src.getType() != XrvType::TypeStruct) {
            return false;
        }
        for (XrvType::ValueStruct::const_iterator it = const_cast<XrvType &>(src).begin();
             it != const_cast<XrvType &>(src).end(); ++it) {
            if (it->first == "topic") {
                Helper<std::string>::cast(it->second, dst.topic);
            } else if (it->first == "transport") {
                Helper<std::string>::cast(it->second, dst.transport);
            } else if (it->first == "aoi") {
                Helper<std::vector<cv::Vec2f> >::cast(it->second, dst.aoi);
            } else if (it->first == "transform") {
                Helper<std::vector<cv::Vec3f> >::cast(it->second, dst.transform);
            } else if (it->first == "transparency") {
                Helper<double>::cast(it->second, dst.transparency);
            } else if (it->first == "primary") {
                Helper<bool>::cast(it->second, dst.primary);
            }
        }
        return true;
    }

    static void cast(const ValueType &src, XrvType &dst) {
        dst.clear();
        // this must be called to make dst a struct type
        // dst.begin();
        Helper<std::string>::cast(src.topic, dst["topic"]);
        Helper<std::string>::cast(src.transport, dst["transport"]);
        Helper<std::vector<cv::Vec2f> >::cast(src.aoi, dst["aoi"]);
        Helper<std::vector<cv::Vec3f> >::cast(src.transform, dst["transform"]);
        Helper<double>::cast(src.transparency, dst["transparency"]);
        Helper<bool>::cast(src.primary, dst["primary"]);
    }

    static bool get(const std::string &name, ValueType &val) {
        XrvType xrv;
        if (!ros::param::get(name, xrv)) {
            return false;
        }
        if (!ThisType::cast(xrv, val)) {
            return false;
        }
        return true;
    }

    static void set(const std::string &name, const ValueType &val) {
        XrvType xrv;
        ThisType::cast(val, xrv);
        ros::param::set(name, xrv);
    }

    static bool write(const ValueType &val, std::ostream &ost) {
        XrvType xrv;
        ThisType::cast(val, xrv);
        xrv.write(ost);
        return true;
    }
};
}
}

#endif  // IMAGE_REPROJECTION_TOOLS_IMAGE_SUPERIMPOSITION_PARAMS_HPP