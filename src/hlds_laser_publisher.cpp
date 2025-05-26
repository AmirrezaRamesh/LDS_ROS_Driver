#include <rclcpp/rclcpp.hpp>
 #include <sensor_msgs/msg/laser_scan.hpp>
 #include <boost/asio.hpp>
 #include <hls_lfcd_lds_driver/lfcd_laser.hpp>
 
 namespace hls_lfcd_lds
 {
 LFCDLaser::LFCDLaser(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
   : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
 {
   serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
 
   boost::asio::write(serial_, boost::asio::buffer("b", 1));  // start motor
 }
 
 LFCDLaser::~LFCDLaser()
 {
   boost::asio::write(serial_, boost::asio::buffer("e", 1));  // stop motor
 }
 
 void LFCDLaser::poll(sensor_msgs::msg::LaserScan::SharedPtr scan, int range)
 {
   uint8_t start_count = 0;
   bool got_scan = false;
   boost::array<uint8_t, 2520> raw_bytes;
   uint8_t good_sets = 0;
   uint32_t motor_speed = 0;
   rpms = 0;
   int index;
 
 
 
   while (!shutting_down_ && !got_scan)
   {
     boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[start_count], 1));
 
     if (start_count == 0)
     {
       if (raw_bytes[start_count] == 0xFA)
       {
         start_count = 1;
       }
     }
     else if (start_count == 1)
     {
       if (raw_bytes[start_count] == 0xA0)
       {
         start_count = 0;
 
         got_scan = true;
 
         boost::asio::read(serial_, boost::asio::buffer(&raw_bytes[2], 2518));

        //**************************************Topic Message ****************************************************/
         scan->angle_increment = (M_PI / 180.0);

         int half_angle = range / 2;
         int start_index = 0 - half_angle;
         int end_index = 0 + half_angle;
         if (start_index < 0) start_index += 360;
         if (end_index > 359) end_index -= 360;

         scan->angle_min = start_index * scan->angle_increment;
         scan->angle_max = (end_index - 1) * scan->angle_increment;
         scan->range_min = 0.12;
         scan->range_max = 3.5;
         scan->ranges.resize(range);  
         scan->intensities.resize(range);
        //********************************************************************************************************/
         for (uint16_t i = 0; i < raw_bytes.size(); i = i + 42)
         {
           if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) 
           {
             good_sets++;
             motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]; 
 
             for (uint16_t j = i + 4; j < i + 40; j = j + 6)
             {
               index = 6 * (i / 42) + (j - 4 - i) / 6;
 
               uint8_t byte0 = raw_bytes[j];
               uint8_t byte1 = raw_bytes[j + 1];
               uint8_t byte2 = raw_bytes[j + 2];
               uint8_t byte3 = raw_bytes[j + 3];
 
               uint16_t intensity = (byte1 << 8) + byte0;
               uint16_t range = (byte3 << 8) + byte2;
 
               int corrected_index = 359 - index;
               if (corrected_index >= start_index && corrected_index < end_index)
               {
                 int clipped_index = corrected_index - start_index;
                 scan->ranges[clipped_index] = range / 1000.0;
                 scan->intensities[clipped_index] = intensity;
               }
             }
           }
         }
 
         rpms = motor_speed / good_sets / 10;
         scan->time_increment = (float)(1.0 / (rpms * 6));
         scan->scan_time = scan->time_increment * range;
       }
       else
       {
         start_count = 0;
       }
     }
   }
 }
 

 int main(int argc, char **argv)
 {
   rclcpp::init(argc, argv);
 
   auto node = rclcpp::Node::make_shared("hlds_laser_publisher");
   rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
   boost::asio::io_service io;
 
   std::string port;
   std::string frame_id;
   int baud_rate;
   int range;
 
   node->declare_parameter<std::string>("port");
   node->declare_parameter<std::string>("frame_id");
   node->declare_parameter<int>("range"); 
 
   node->get_parameter_or<std::string>("port", port, "/dev/ttyUSB0");
   node->get_parameter_or<std::string>("frame_id", frame_id, "laser");
   node->get_parameter_or<std::string>("range", range, 360);

 
   baud_rate = 230400;
 
   RCLCPP_INFO(node->get_logger(), "Init hlds_laser_publisher Node Main");
   RCLCPP_INFO(node->get_logger(), "port : %s frame_id : %s range : %d", port.c_str(), frame_id.c_str(), range);
 
   try
   {
     hls_lfcd_lds::LFCDLaser laser(port, baud_rate, io);
     laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(rclcpp::SensorDataQoS()));
 
     while (rclcpp::ok())
     {
       auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
       scan->header.frame_id = frame_id;
       laser.poll(scan, range);
       scan->header.stamp = node->now();
       laser_pub->publish(*scan);
     }
     laser.close();
 
     return 0;
   }
   catch (boost::system::system_error& ex)
   {
     return -1;
   }
 }
