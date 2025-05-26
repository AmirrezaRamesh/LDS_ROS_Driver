
#ifndef HLDS_LASER_SEGMENT_PUBLISHER_H_
#define HLDS_LASER_SEGMENT_PUBLISHER_H_

#include <string>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/asio.hpp>
#include <boost/array.hpp>

namespace hls_lfcd_lds
{
class LFCDLaser
{
 public:
  //uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an LFCD packet
  /**
  * @brief Constructs a new LFCDLaser attached to the given serial port
  * @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
  * @param baud_rate The baud rate to open the serial port at.
  * @param io Boost ASIO IO Service to use when creating the serial port object
  */
	LFCDLaser(boost::asio::io_service& io);

  /**
  * @brief Default destructor
  */
  ~LFCDLaser() {};

  /**
  * @brief Poll the laser to get a new scan. Blocks until a complete new scan is received or close is called.
  * @param scan LaserScan message pointer to fill in with the scan. The caller is responsible for filling in the ROS timestamp and frame_id
  */
  void poll();

  /**
  * @brief Close the driver down and prevent the polling loop from advancing
  */
  void close();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;

  // ROS Topic Publishers
  ros::Publisher laser_pub_;

  std::string frame_id_;
  sensor_msgs::LaserScan scan_;

  std::string port_; ///< @brief The serial port the driver is attached to
  int baud_rate_; ///< @brief The baud rate for the serial connection
  int lfcdstart_;
  int lfcdstop_;
  int lfcdstartstop_; ///< @brief The LDS Start/Stop to check.
  bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
  boost::asio::serial_port serial_; ///< @brief Actual serial port object for reading/writing to the LFCD Laser Scanner
  uint16_t motor_speed_; ///< @brief current motor speed as reported by the LFCD.
};
};

#endif // HLDS_LASER_SEGMENT_PUBLISHER_H_
