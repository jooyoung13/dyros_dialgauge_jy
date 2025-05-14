#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <fcntl.h>  // for fcntl, O_NONBLOCK

using namespace boost::asio;

class SerialNode {
public:
  SerialNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh),
      pnh_(pnh),
      io_(),
      serial_(io_),
      sending_(false)
  {
    pnh_.param("port", port_, std::string("/dev/ttyUSB0"));
    pnh_.param("baud", baud_, 2400);

    open_srv_  = pnh_.advertiseService("open",  &SerialNode::openPort,     this);
    start_srv_ = pnh_.advertiseService("start", &SerialNode::startPublishing, this);
    stop_srv_  = pnh_.advertiseService("stop",  &SerialNode::stopPublishing, this);

    pub_ = nh_.advertise<std_msgs::Float64>("/dial_gauge/raw", 1000);
  }

private:
  // --- Service callbacks ---
  bool openPort(std_srvs::SetBool::Request& req,
                std_srvs::SetBool::Response& res)
  {
    if (!req.data) {
      if (serial_.is_open()) {
        serial_.close();
        res.success = true;
        res.message = "Port closed";
        ROS_INFO("Port closed");
      } else {
        res.success = false;
        res.message = "Port not open";
      }
      return true;
    }

    // req.data == true: open
    if (serial_.is_open()) {
      res.success = false;
      res.message = "Already open";
    } else {
      try {
        serial_.open(port_);
        serial_.set_option(serial_port_base::baud_rate(baud_));
        // set nonblocking (optional)
        int fd = serial_.native_handle();
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);

        res.success = true;
        res.message = "Port opened";
        ROS_INFO("Port opened: %s", port_.c_str());
      } catch (std::exception& e) {
        res.success = false;
        res.message = e.what();
        ROS_ERROR("Failed to open port: %s", e.what());
      }
    }
    return true;
  }

  bool startPublishing(std_srvs::Trigger::Request& req,
                       std_srvs::Trigger::Response& res)
  {
    if (!serial_.is_open()) {
      res.success = false;
      res.message = "Port not open";
      return true;
    }
    if (sending_) {
      res.success = false;
      res.message = "Already publishing";
      return true;
    }

    sending_ = true;

    // start async read
    startAsyncRead();

    // run io_service in background
    io_thread_ = boost::thread([this]{ io_.run(); });

    // start writer thread
    writer_thread_ = boost::thread(boost::bind(&SerialNode::writeLoop, this));

    res.success = true;
    res.message = "Publishing started";
    ROS_INFO("Publishing started");
    return true;
  }

  bool stopPublishing(std_srvs::Trigger::Request& req,
                      std_srvs::Trigger::Response& res)
  {
    if (!sending_) {
      res.success = false;
      res.message = "Not publishing";
      return true;
    }

    sending_ = false;

    // stop async operations
    io_.stop();
    // cancel outstanding operations
    try { serial_.cancel(); } catch(...) {}

    // join threads
    if (writer_thread_.joinable()) writer_thread_.join();
    if (io_thread_.joinable())     io_thread_.join();

    // reset io_service for next start
    io_.reset();

    res.success = true;
    res.message = "Publishing stopped";
    ROS_INFO("Publishing stopped");
    return true;
  }

  // --- Asynchronous read loop setup ---
  void startAsyncRead() {
    serial_.async_read_some(
      buffer(&read_buf_, 1),
      boost::bind(&SerialNode::handleRead,
                  this,
                  placeholders::error,
                  placeholders::bytes_transferred)
    );
  }

  void handleRead(const boost::system::error_code& ec, std::size_t bytes_transferred) {
    if (!sending_ || ec) {
      // either stopped or error
      return;
    }

    char c = read_buf_;
    // collect line
    if (c!='\n' && c!='\r') {
      line_buf_ += c;
      // read next byte
      startAsyncRead();
      return;
    }

    // parse line_buf_
    size_t pos = line_buf_.find_last_of("+-");
    if (pos != std::string::npos) {
      size_t end = pos+1;
      while (end < line_buf_.size() &&
             (isdigit(line_buf_[end]) || line_buf_[end]=='.'))
        ++end;
      std::string num = line_buf_.substr(pos, end-pos);
      try {
        double v = std::stod(num);
        std_msgs::Float64 msg; msg.data = v;
        pub_.publish(msg);
        ROS_INFO_STREAM("Value: " << v);
      } catch(...) {
        ROS_WARN("Failed to parse '%s'", num.c_str());
      }
    }
    line_buf_.clear();
    // queue next read
    startAsyncRead();
  }

  // --- Synchronous write loop ---
  void writeLoop() {
    ROS_INFO("writeLoop started");
    int fd = serial_.native_handle();
    int dtr = TIOCM_DTR;
    const boost::posix_time::milliseconds dtr_low(10);
    const boost::posix_time::milliseconds interval(80);

    while (sending_ && ros::ok()) {
      ioctl(fd, TIOCMBIC, &dtr);
      boost::this_thread::sleep(dtr_low);
      ioctl(fd, TIOCMBIS, &dtr);
      write(serial_, buffer(std::string("\r")));
      boost::this_thread::sleep(interval);
    }
    ROS_INFO("writeLoop exiting");
  }

  // --- Members ---
  ros::NodeHandle       nh_, pnh_;
  ros::Publisher        pub_;
  ros::ServiceServer    open_srv_, start_srv_, stop_srv_;
  std::string           port_;
  int                   baud_;

  io_service            io_;
  serial_port           serial_;
  boost::thread         io_thread_, writer_thread_;

  bool                  sending_;
  char                  read_buf_;
  std::string           line_buf_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh, pnh("~");
  SerialNode node(nh, pnh);
  ros::spin();
  return 0;
}
