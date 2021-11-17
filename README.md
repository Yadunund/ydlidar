YDLIDAR ROS 2 FOXY DRIVER
=====================================================================

ROS 2 node and test application for YDLIDAR X2L.

Visit EAI Website for more details about YDLIDAR.

BUILD
=====================================================================
```
git clone https://github.com/Yadunund/ydlidar_ros.git
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to ydlidar
```

RUN
=====================================================================
```
ros2 launch ydlidar lidar.launch.xml
```


PARAMETERS
=====================================================================
port (string, default: /dev/ydlidar)

    serial port name used in your system.

frequency (double, default: 8.0)
    
    publish frequency in Hz
    
topic (string, default: scan)

    topic to publish LaserScan messages

frame_id (string, default: laser_frame)

    frame ID for the device.

reversion (bool, default: false)

    indicated whether the LIDAR IS reversion.

sun_noise (bool, default: true)

    indicated whether to turn off solar noise

glass_noise (bool, default: true)

    indicated whether to turn off glass noise.

resolution_fixed (bool, default: true)

    indicated whether the LIDAR has a fixed angular resolution.

angle_min (double, default: -180)

    Min valid angle (°) for LIDAR data.

angle_max (double, default: 180)

    Max valid angle (°) for LIDAR data.

range_min (double, default: 0.08)

    Min valid range (m) for LIDAR data.

range_max (double, default: 12.0)

    Max valid range (m) for LIDAR data.

ignore_array (string, default: "")

    Set the current angle range value to zero.

max_abnormal_check_count (int, default: 2)

    the LIDAR scanning the maximum number of abnormal checks.


