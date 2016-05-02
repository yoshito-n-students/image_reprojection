#ifndef _UTILITY_HEADERS_VERBOSE_CONSOLE_HPP_
#define _UTILITY_HEADERS_VERBOSE_CONSOLE_HPP_

#include <sstream>

#include <ros/console.h>
#include <ros/this_node.h>

#include <boost/filesystem/path.hpp>

namespace utility_headers{
  namespace console{

    class VerboseFilter:
      public ros::console::FilterBase{
    private:
      VerboseFilter()
	:level_node_(ros::console::levels::Debug),
	 level_function_(ros::console::levels::Error),
	 level_file_(ros::console::levels::Error){
      }

      VerboseFilter(const VerboseFilter &);

      VerboseFilter & operator=(const VerboseFilter &);

      virtual ~VerboseFilter(){
      }

    public:
      virtual bool isEnabled(){
	return true;
      }

      virtual bool isEnabled(ros::console::FilterParams & param){
	namespace bf = boost::filesystem;
	const bool show_node(param.level >= level_node_);
	const bool show_function(param.level >= level_function_);
	const bool show_file(param.level >= level_file_);
	std::ostringstream oss;
	if(show_node){
	  oss << ros::this_node::getName() << ": ";
	}
	oss << param.message;
	if(show_function || show_file){
	  oss << " (";
	  if(show_function){
	    oss << param.function;
	  }
	  if(show_file){
	    oss << "@" << bf::path(param.file).filename().string()
		<< ":" << param.line;
	  }
	  oss << ")";
	}
	param.out_message = oss.str();
	return true;
      }

    public:
      static VerboseFilter * ptr(){
	static VerboseFilter self;
	return &self;
      }
      
      static void showNode(const ros::console::Level & level){
	ptr()->level_node_ = level;
      }
      
      static void showFunction(const ros::console::Level & level){
	ptr()->level_function_ = level;
      }
      
      static void showFile(const ros::console::Level & level){
	ptr()->level_file_ = level;
      }
      
    private:
      ros::console::Level level_node_;
      ros::console::Level level_function_;
      ros::console::Level level_file_;
    };

  }
}


//
// printf-style verbose console macros
//

#define ROS_VDEBUG(...)							\
  ROS_DEBUG_FILTER(::utility_headers::console::VerboseFilter::ptr(),__VA_ARGS__)

#define ROS_VINFO(...)							\
  ROS_INFO_FILTER(::utility_headers::console::VerboseFilter::ptr(),__VA_ARGS__)

#define ROS_VWARN(...)							\
  ROS_WARN_FILTER(::utility_headers::console::VerboseFilter::ptr(),__VA_ARGS__)

#define ROS_VERROR(...)							\
  ROS_ERROR_FILTER(::utility_headers::console::VerboseFilter::ptr(),__VA_ARGS__)

#define ROS_VFATAL(...)							\
  ROS_FATAL_FILTER(::utility_headers::console::VerboseFilter::ptr(),__VA_ARGS__)


//
// stream-style verbose console macros
//
//   In ROS jade, there is an elementary BUG such that the 2nd arg 
//   of ROS_X_STREAM_FILTER does not pass the filter (see rosconsole.cpp).
//   Thus we should call ROS_X_FILTER instead.
//

#define ROS_VDEBUG_STREAM(args)						\
  do{									\
    /* osstream with a meanigless long suffix */			\
    /* that should not hide any variable names in args */		\
    std::ostringstream oss_ghuirsghrisguhrgiuhsr;			\
    oss_ghuirsghrisguhrgiuhsr << args;					\
    ROS_VDEBUG("%s",oss_ghuirsghrisguhrgiuhsr.str().c_str());		\
  }									\
  while(0)

#define ROS_VINFO_STREAM(args)						\
  do{									\
    std::ostringstream oss_ghuirsghrisguhrgiuhsr;			\
    oss_ghuirsghrisguhrgiuhsr << args;					\
    ROS_VINFO("%s",oss_ghuirsghrisguhrgiuhsr.str().c_str());		\
  }									\
  while(0)

#define ROS_VWARN_STREAM(args)						\
  do{									\
    std::ostringstream oss_ghuirsghrisguhrgiuhsr;			\
    oss_ghuirsghrisguhrgiuhsr << args;					\
    ROS_VWARN("%s",oss_ghuirsghrisguhrgiuhsr.str().c_str());		\
  }									\
  while(0)

#define ROS_VERROR_STREAM(args)						\
  do{									\
    std::ostringstream oss_ghuirsghrisguhrgiuhsr;			\
    oss_ghuirsghrisguhrgiuhsr << args;					\
    ROS_VERROR("%s",oss_ghuirsghrisguhrgiuhsr.str().c_str());		\
  }									\
  while(0)

#define ROS_VFATAL_STREAM(args)						\
  do{									\
    std::ostringstream oss_ghuirsghrisguhrgiuhsr;			\
    oss_ghuirsghrisguhrgiuhsr << args;					\
    ROS_VFATAL("%s",oss_ghuirsghrisguhrgiuhsr.str().c_str());		\
  }									\
  while(0)

#endif /* _UTILITY_HEADERS_VERBOSE_CONSOLE_HPP_ */
