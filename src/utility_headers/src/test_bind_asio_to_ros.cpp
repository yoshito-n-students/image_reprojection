#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>

#include <ros/console.h>
#include <ros/init.h>
#include <ros/node_handle.h>

#include "utility_headers/bind_asio_to_ros.hpp"

class asio_printer{
public:
  asio_printer(boost::asio::io_service & asio_queue)
    :timer_(asio_queue),count_(0),start_time_(ros::Time::now()){
    start();
  }

  virtual ~asio_printer(){
  }

private:
  void start(){
    timer_.expires_from_now(ros::Duration(0.1).toBoost());
    timer_.async_wait(boost::bind(&asio_printer::handle,this,_1));
  }

  void handle(const boost::system::error_code & error){
    ROS_INFO_STREAM("Printer called "
		    << "(#" << ++count_ << ": "
		    << (ros::Time::now() - start_time_).toSec()
		    << "s elapsed)");
    start();
  }

private:
  boost::asio::deadline_timer timer_;
  std::size_t count_;
  const ros::Time start_time_;
};

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"test_bind_asio_to_ros");
  ros::NodeHandle handle;

  boost::asio::io_service asio_queue;
  utility_headers::bindAsioToRos(asio_queue);

  asio_printer printer(asio_queue);

  ros::spin();

  return 0;
}
