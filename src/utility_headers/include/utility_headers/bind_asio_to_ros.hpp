#ifndef _UTILITY_HEADERS_BIND_ASIO_TO_ROS_HPP_
#define _UTILITY_HEADERS_BIND_ASIO_TO_ROS_HPP_

#include <ros/callback_queue.h>

#include <boost/asio/io_service.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace utility_headers{

  class AsioCallback
    :public ros::CallbackInterface,
     public boost::enable_shared_from_this<AsioCallback>{
  public:
    AsioCallback(boost::asio::io_service & asio_queue,
		 ros::CallbackQueue * ros_queue)
      :asio_queue_(asio_queue),ros_queue_(ros_queue){
    }

    virtual ~AsioCallback(){
    }

  protected:
    virtual CallResult call(){
      if(asio_queue_.run_one() > 0){
	if(ros_queue_){
	  ros_queue_->addCallback(shared_from_this());
	}
      }
      return Success;
    }

  private:
    boost::asio::io_service & asio_queue_;
    ros::CallbackQueue * const ros_queue_;
  };

  static inline void bindAsioToRos(boost::asio::io_service & asio_queue,
				   ros::CallbackQueue * ros_queue
				   = ros::getGlobalCallbackQueue()){
    boost::shared_ptr<AsioCallback> ptr(new AsioCallback(asio_queue,ros_queue));
    ros_queue->addCallback(ptr);
  }

}

#endif /* _UTILITY_HEADERS_BIND_ASIO_TO_ROS_HPP_ */
