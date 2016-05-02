#ifndef _UTILITY_HEADERS_PARAM_HPP_
#define _UTILITY_HEADERS_PARAM_HPP_

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <ros/exception.h>
#include <ros/node_handle.h>
#include <ros/param.h>

#include <utility_headers/verbose_console.hpp>

#include <XmlRpcValue.h>

#include <boost/array.hpp>
#include <boost/type_traits/is_floating_point.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/utility/enable_if.hpp>

namespace utility_headers {
namespace param {

//
// Helper functions for specific fundermental/array/collection type
//  - casts between XmlRpc and the specific type
//  - get/set a parameter of the specific type
//  - write a paramater of the specific type to a stream
//

template <typename T, typename EnableIf = void>
struct Helper;

// for bool
template <>
struct Helper<bool> {
    typedef bool ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        if (src.getType() != XrvType::TypeBoolean) {
            return false;
        }
        dst = const_cast<XrvType &>(src);
        return true;
    }

    static void cast(const ValueType &src, XrvType &dst) { dst = src; }

    static bool get(const std::string &name, ValueType &val) { return ros::param::get(name, val); }

    static void set(const std::string &name, const ValueType &val) { ros::param::set(name, val); }

    static bool write(const ValueType &val, std::ostream &ost) { ost << val; }
};

// for integral
template <typename T>
struct Helper<T, typename boost::enable_if<boost::is_integral<T> >::type> {
    typedef T ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        switch (src.getType()) {
            case XrvType::TypeInt: {
                dst = static_cast<int>(const_cast<XrvType &>(src));
                return true;
            }
            case XrvType::TypeDouble: {
                dst = static_cast<double>(const_cast<XrvType &>(src));
                return true;
            }
            default: { return false; }
        }
    }

    static void cast(const ValueType &src, XrvType &dst) { dst = static_cast<int>(src); }

    static bool get(const std::string &name, ValueType &val) {
        int int_val;
        const bool ret(ros::param::get(name, int_val));
        val = static_cast<ValueType>(int_val);
        return ret;
    }

    static void set(const std::string &name, const ValueType &val) { ros::param::set(name, val); }

    static bool write(const ValueType &val, std::ostream &ost) { ost << val; }
};

// for floating point
template <typename T>
struct Helper<T, typename boost::enable_if<boost::is_floating_point<T> >::type> {
    typedef T ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        switch (src.getType()) {
            case XrvType::TypeInt: {
                dst = static_cast<int>(const_cast<XrvType &>(src));
                return true;
            }
            case XrvType::TypeDouble: {
                dst = static_cast<double>(const_cast<XrvType &>(src));
                return true;
            }
            default: { return false; }
        }
    }

    static void cast(const ValueType &src, XrvType &dst) { dst = static_cast<double>(src); }

    static bool get(const std::string &name, ValueType &val) {
        double double_val;
        const bool ret(ros::param::get(name, double_val));
        val = static_cast<ValueType>(double_val);
        return ret;
    }

    static void set(const std::string &name, const ValueType &val) { ros::param::set(name, val); }

    static bool write(const ValueType &val, std::ostream &ost) { ost << val; }
};

// for std::string
template <>
struct Helper<std::string> {
    typedef std::string ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        if (src.getType() != XrvType::TypeString) {
            return false;
        }
        dst = static_cast<ValueType &>(const_cast<XrvType &>(src));
        return true;
    }

    static void cast(const ValueType &src, XrvType &dst) { dst = src; }

    static bool get(const std::string &name, ValueType &val) { return ros::param::get(name, val); }

    static void set(const std::string &name, const ValueType &val) { ros::param::set(name, val); }

    static bool write(const ValueType &val, std::ostream &ost) { ost << val; }
};

// for std::vector
template <typename T, typename A>
struct Helper<std::vector<T, A> > {
    typedef Helper<typename std::vector<T, A> > ThisType;
    typedef typename std::vector<T, A> ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        if (src.getType() != XrvType::TypeArray) {
            return false;
        }
        dst.resize(src.size());
        for (std::size_t i = 0; i < src.size(); ++i) {
            if (!Helper<T>::cast(src[i], dst[i])) {
                return false;
            }
        }
        return true;
    }

    static void cast(const ValueType &src, XrvType &dst) {
        dst.clear();
        // this must be called to make dst a array type
        // in case the following for does not loop
        dst.setSize(src.size());
        for (std::size_t i = 0; i < src.size(); ++i) {
            Helper<T>::cast(src[i], dst[i]);
        }
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

// for boost::array
template <typename T, std::size_t N>
struct Helper<boost::array<T, N> > {
    typedef Helper<typename boost::array<T, N> > ThisType;
    typedef typename boost::array<T, N> ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        if (src.getType() != XrvType::TypeArray) {
            return false;
        }
        if (src.size() != N) {
            return false;
        }
        for (std::size_t i = 0; i < N; ++i) {
            if (!Helper<T>::cast(src[i], dst[i])) {
                return false;
            }
        }
        return true;
    }

    static void cast(const ValueType &src, XrvType &dst) {
        dst.clear();
        // this must be called to make dst a array type
        // in case the following for does not loop
        dst.setSize(N);
        for (std::size_t i = 0; i < N; ++i) {
            Helper<T>::cast(src[i], dst[i]);
        }
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

// for std::map
template <typename T, typename C, typename A>
struct Helper<std::map<std::string, T, C, A> > {
    typedef Helper<typename std::map<std::string, T, C, A> > ThisType;
    typedef typename std::map<std::string, T, C, A> ValueType;
    typedef XmlRpc::XmlRpcValue XrvType;

    static bool cast(const XrvType &src, ValueType &dst) {
        if (src.getType() != XrvType::TypeStruct) {
            return false;
        }
        dst.clear();
        for (XrvType::ValueStruct::const_iterator it = const_cast<XrvType &>(src).begin();
             it != const_cast<XrvType &>(src).end(); ++it) {
            typename ValueType::value_type elm(it->first, T());
            if (!Helper<T>::cast(it->second, elm.second)) {
                return false;
            }
            if (!dst.insert(elm).second) {
                return false;
            }
        }
        return true;
    }

    static void cast(const ValueType &src, XrvType &dst) {
        dst.clear();
        // this must be called to make dst a struct type
        // in case the following for does not loop
        dst.begin();
        for (typename ValueType::const_iterator it = src.begin(); it != src.end(); ++it) {
            Helper<T>::cast(it->second, dst[it->first]);
        }
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

//
// Utility functions to get (and print) a parameter
//

template <typename T>
T param(const std::string &name, const T &default_val, const bool do_print = true) {
    T val;
    const bool could_get(Helper<T>::get(name, val));
    if (do_print) {
        std::ostringstream oss;
        oss << name << " = ";
        if (could_get) {
            Helper<T>::write(val, oss);
        } else {
            Helper<T>::write(default_val, oss);
            oss << " (default)";
        }
        ROS_VINFO_STREAM(oss.str());
    }
    return could_get ? val : default_val;
}

template <typename T>
T param(const ros::NodeHandle &handle, const std::string &name, const T &default_val,
        const bool do_print = true) {
    return param(handle.resolveName(name), default_val, do_print);
}

template <typename T>
bool get(const std::string &name, T &val, const bool do_print = true) {
    const bool could_get(Helper<T>::get(name, val));
    if (do_print) {
        std::ostringstream oss;
        oss << name << " = ";
        if (could_get) {
            Helper<T>::write(val, oss);
            ROS_VINFO_STREAM(oss.str());
        } else {
            oss << "(Could not get)";
            ROS_VWARN_STREAM(oss.str());
        }
    }
    return could_get;
}

template <typename T>
bool get(const ros::NodeHandle &handle, const std::string &name, T &val,
         const bool do_print = true) {
    return get(handle.resolveName(name), val, do_print);
}

class NoRequiredParameterException : public ros::Exception {
   public:
    NoRequiredParameterException(const std::string &name)
        : Exception("No required parameter " + name) {}

    virtual ~NoRequiredParameterException() throw() {}
};

template <typename T>
void getRequired(const std::string &name, T &val, const bool do_print = true) {
    if (!get(name, val, do_print)) {
        throw NoRequiredParameterException(name);
    }
}

template <typename T>
void getRequired(const ros::NodeHandle &handle, const std::string &name, T &val,
                 const bool do_print = true) {
    getRequired(handle.resolveName(name), val, do_print);
}

//
// Utility function to set a parameter
//

template <typename T>
void set(const std::string &name, const T &val) {
    Helper<T>::set(name, val);
}

//
// Utility function to delete a parameter
// (just a wrapper of ros::param::del)
//

static inline bool del(const std::string &name) { return ros::param::del(name); }

//
// Utility function to know whether a parameter exists
// (just a wrapper of ros::param::has)
//

static inline bool has(const std::string &name) { return ros::param::has(name); }
}
}

#endif /* _UTILITY_HEADERS_PARAM_HPP_ */
