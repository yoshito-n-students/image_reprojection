#ifndef _UTILITY_HEADERS_LS_HPP_
#define _UTILITY_HEADERS_LS_HPP_

#include <iostream>
#include <stdio.h>
#include <string>
#include <vector>

#include <utility_headers/verbose_console.hpp>

#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/stream.hpp>

namespace utility_headers{
  namespace ls{

    inline std::vector<std::string> ls(const std::string & path){
      namespace bi = boost::iostreams;

      std::vector<std::string> files;

      const std::string command("ls -d -1 " + path);
      FILE * fp(popen(command.c_str(),"r"));
      if(!fp){
	ROS_VERROR_STREAM("Failed to open a pipe to " << "\"" << command << "\"");
	return files;
      }

      bi::stream<bi::file_descriptor_source> st(fileno(fp),bi::never_close_handle);
      std::string file;
      while(std::getline(st,file)){
	files.push_back(file);
      }

      pclose(fp);

      return files;
    }

  }
}

#endif /* _UTILITY_HEADERS_LS_HPP_ */
