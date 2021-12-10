#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "CYdLidar.h"
#include <signal.h>

#include <vector>
#include <iostream>
#include <string>

using namespace ydlidar;

class LidarNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using LaserScanMsg = sensor_msgs::msg::LaserScan;
  using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  // Constructor
  LidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("ydlidar_lifecycle_node", options)
  {

    RCLCPP_INFO(get_logger(), "Starting lifecycle ydlidar node");
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Declare node parameters
    _port = this->declare_parameter("port", "/dev/ttyUSB0");
    _frequency = this->declare_parameter("frequency", 8.0);
    _topic = this->declare_parameter("topic", "scan");
    _frame_id = this->declare_parameter("frame_id", "laser_frame");
    _resolution_fixed = this->declare_parameter("resolution_fixed", true);
    _auto_reconnect = this->declare_parameter("auto_reconnect", true);
    _sun_noise = this->declare_parameter("sun_noise", true);
    _glass_noise = this->declare_parameter("glass_noise", true);
    _reversion = this->declare_parameter("reversion", false);
    _angle_max = this->declare_parameter("angle_max", 180.0);
    _angle_min = this->declare_parameter("angle_min", -180.0);
    _max_range = this->declare_parameter("range_max", 12.0);
    _min_range = this->declare_parameter("range_min", 0.08);
    _OffsetTime = this->declare_parameter("OffsetTime", 0.0);
    _max_abnormal_check_count = this->declare_parameter("max_abnormal_check_count", 2);
    _list = this->declare_parameter("ignore_array", "");
    _ignore_array = split(_list, ',');
    if(_ignore_array.size() % 2)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "ignore array is odd need be even");
    }

    for(uint16_t i = 0 ; i < _ignore_array.size(); i++)
    {
      if(_ignore_array[i] < -180 && _ignore_array[i] > 180)
      {
        RCLCPP_ERROR(
          this->get_logger(),
          "ignore array should be between -180 and 180");
      }
    }

    if(_angle_max < _angle_min)
    {
      double temp = _angle_max;
      _angle_max = _angle_min;
      _angle_min = temp;
    }
    if (_max_abnormal_check_count < 2)
    {
      _max_abnormal_check_count = 2;
    }

    // Set laser parameters
    _laser.setSerialPort(_port);
    _laser.setSerialBaudrate(_baudrate);
    _laser.setMaxRange(_max_range);
    _laser.setMinRange(_min_range);
    _laser.setMaxAngle(_angle_max);
    _laser.setMinAngle(_angle_min);
    _laser.setReversion(_reversion);
    _laser.setAutoReconnect(_auto_reconnect);
    _laser.setSunNoise(_sun_noise);
    _laser.setGlassNoise(_glass_noise);
    _laser.setAbnormalCheckCount(_max_abnormal_check_count);
    _laser.setIgnoreArray(_ignore_array);
    _laser.setOffsetTime(_OffsetTime);

    _laser.checkCOMMs();
    _laser.turnOn();
  }

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State&)
  {


    _laser_scan_pub = this->create_publisher<LaserScanMsg>(
      _topic,
      rclcpp::SensorDataQoS());

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::ratio<1>>(1.0/_frequency));

    _timer = this->create_wall_timer(
      period,
      [&]()
      {
        if (!_initialized || !rclcpp::ok() || !_laser_scan_pub->is_activated())
          return;

        bool hardError;
        LaserScan scan;
        if (_laser.doProcessSimple(scan, hardError))
        {
          LaserScanMsg scan_msg;
          // builtin_interfaces::msg::Time start_scan_time;
          // start_scan_time.sec = scan.system_time_stamp/1000000000ul;
          // start_scan_time.nanosec = scan.system_time_stamp%1000000000ul;
          scan_msg.header.stamp = this->now();
          scan_msg.header.frame_id = _frame_id;
          scan_msg.angle_min =(scan.config.min_angle);
          scan_msg.angle_max = (scan.config.max_angle);
          scan_msg.scan_time = scan.config.scan_time;
          scan_msg.time_increment = scan.config.time_increment;
          scan_msg.range_min = (scan.config.min_range);
          scan_msg.range_max = (scan.config.max_range);
          int fixed_size = scan.data.size();
          if (_resolution_fixed)
          {
            fixed_size = _laser.getFixedSize();
          }
          if (scan.config.max_angle - scan.config.min_angle == 2*M_PI)
          {
            scan_msg.angle_increment = (scan.config.max_angle - scan.config.min_angle) / (fixed_size);
          }
          else
          {
            scan_msg.angle_increment = (scan.config.max_angle - scan.config.min_angle) / (fixed_size - 1);
          }
          int index = 0;
          scan_msg.ranges.resize(fixed_size, std::numeric_limits<float>::infinity());
          scan_msg.intensities.resize(fixed_size, 0);

          for(int i = 0; i < scan.data.size(); i++)
          {
            LaserPoint point = scan.data[i];
            index = (point.angle - scan.config.min_angle ) / scan_msg.angle_increment + 0.5;
            if(index >=0 && index < fixed_size)
            {
              if(point.range == 0.0)
              {
                scan_msg.ranges[index] = std::numeric_limits<float>::infinity();
                scan_msg.intensities[index] = 0;
              }
              else
              {
                scan_msg.ranges[index] = point.range;
                scan_msg.intensities[index] = point.intensity;
              }
            }
          }
          _laser_scan_pub->publish(scan_msg);
        }
      });

    RCLCPP_INFO(get_logger(), "ydlidar on_configure() successful.");
    return LifecycleCallbackReturn::SUCCESS;
  }

  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State&)
  {
    _initialized = _laser.initialize();
    if (!_initialized)
    {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to start scan mode!!!");
        return LifecycleCallbackReturn::FAILURE;
    }
    _laser_scan_pub->on_activate();
    return LifecycleCallbackReturn::SUCCESS;
  }

  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State&)
  {
    _laser_scan_pub->on_deactivate();
    _initialized = false;
    _laser.turnOffMotor();
    RCLCPP_INFO(get_logger(), "ydlidar on_deactivate() successful.");
    return LifecycleCallbackReturn::SUCCESS;
  }

  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State&)
  {
    _timer.reset();
    _laser_scan_pub.reset();
    _initialized = false;
    RCLCPP_INFO(get_logger(), "ydlidar on_cleanup() successful.");
    return LifecycleCallbackReturn::SUCCESS;
  }

  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State& state)
  {
    on_cleanup(state);
    _laser.turnOff();
    RCLCPP_INFO(
      this->get_logger(),
      "YDLIDAR is stopping .......");
    RCLCPP_INFO(get_logger(), "ydlidar on_shutdown() successful.");
    return LifecycleCallbackReturn::SUCCESS;
  }

  std::vector<float> split(const std::string &s, char delim)
  {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
  }

  // ~LidarNode()
  // {
  //   _laser.turnOff();
  //   RCLCPP_INFO(
  //     this->get_logger(),
  //     "YDLIDAR is stopping .......");
  //   _laser.disconnecting();
  // }

private:
  std::string _port;
  double _frequency;
  std::string _topic;
  int _baudrate = 115200;
  std::string _frame_id;
  bool _reversion;
  bool _resolution_fixed;
  bool _auto_reconnect;
  double _angle_max;
  double _angle_min;
  result_t _op_result;
  std::string _list;
  std::vector<float> _ignore_array;
  double _max_range;
  double _min_range;
  bool _sun_noise;
  bool _glass_noise;
  int _max_abnormal_check_count;
  double _OffsetTime = 0.0;
  bool _initialized = false;

  CYdLidar _laser;

  // LaserScan publihser
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<LaserScanMsg>> _laser_scan_pub;
  // Wall timer
  rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char* argv[])
{
  const std::vector<std::string> args =
    rclcpp::init_and_remove_ros_arguments(argc, argv);

  const auto node = std::make_shared<LidarNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
