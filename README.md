# Modified LDS 1.0 ROS Driver 
I added some small changes to LDS ROS2 driver which I initially forked from the official github repo. 

### Setup : 
- Laser Scanner : LDS 1.0
- ROS2 Distro : Humble
- OS : ubuntu 22.04 LTS

### Intro : 
here we have a cpp program which will receive the data over a serial port and publishes the data on a /scan topic with the predefined QoS profile for sensor data
```
laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan",rclcpp::QoS(rclcpp::SensorDataQoS()));
```
When you launch the program, it publishes data for the full 360 degrees of the surrounding environment. However, sometimes our LiDAR is positioned on the robot in a way that certain angles are blocked by the robot's body. Or, in some scenarios where the robot needs to navigate a path, we may not need the full 360-degree data, as parts of it might be irrelevant or unhelpful.

To address this, a parameter called `range` is defined in the launch file hlds_laser.launch.py. This parameter specifies the angular range of data we want to use (measured from the front of the LiDAR/robot). By default, it is set to 360, but you can modify it in the launch file, along with the frame_id and port parameters.

```python
def generate_launch_description():
    #port
    port = LaunchConfiguration('port', default='/dev/ttyUSB0')

    #fram_id of the topic
    frame_id = LaunchConfiguration('frame_id', default='laser')

    #range of data
    range = LaunchConfiguration('range', default= 360)

    return LaunchDescription([

        DeclareLaunchArgument(
            'port',
            default_value=port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar. Default frame_id is \'laser\''),
            
        DeclareLaunchArgument(
            'range',
            default_value=360,
            description='Specifying the range of angles we need the data of, being published on /scan'),
        Node(
            package='hls_lfcd_lds_driver',
            executable='hlds_laser_publisher',
            name='hlds_laser_publisher',
            parameters=[{'port': port, 'frame_id': frame_id, 'range' : range}],
            output='screen'),
    ])

```

how to run : 
0. navigate to src of a ros2 ws : ```cd ros2_ws/src/ ```
1. clone the package: ```git clone https://github.com/AmirrezaRamesh/LDS_ROS_Driver ```
2. back to ws and build: ```colcon build --packges-select LDS_ROS_Driver```
3. source enviroment : ```source ~/ros2_ws/install/setup.bash```
4. run the launch file: ```ros2 launch LDS_ROS_Driver hlds_laser.launch.py```
5. LDS 1.0 is drived and the /scan is published 
