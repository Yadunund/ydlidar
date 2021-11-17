/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS Node Client 
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.ydlidar.com
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#define RAD2DEG(x) ((x)*180./M_PI)

using LaserScan = sensor_msgs::msg::LaserScan;

class ClientNode : public rclcpp::Node
{
public:

  ClientNode(const std::string& node_name)
  : Node(node_name)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Starting node [%]", node_name.c_str());

    _laser_scan_sub = this->create_subscription<LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      [](const LaserScan::SharedPtr msg)
      {
        int count = msg->scan_time / msg->time_increment;
        printf("[YDLIDAR INFO]: I heard a laser scan %s[%d]:\n", msg->header.frame_id.c_str(), count);
        printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(msg->angle_min), RAD2DEG(msg->angle_max));
      
        for(int i = 0; i < count; i++) 
        {
          const float degree = RAD2DEG(msg->angle_min + msg->angle_increment * i);
          if(degree > -5 && degree< 5)
                printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, msg->ranges[i], i);
        }
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
  _laser_scan_sub;
};

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);
  
  const std::string node_name = "ydliar_client";
  rclcpp::spin(std::make_shared<ClientNode>(
      node_name));
  rclcpp::shutdown();

  return 0;
}
