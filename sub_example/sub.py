import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import signal
import sys

class scan_front(Node):

    def __init__(self):
        super().__init__('scan_front_node')

        self.scan_ranges = []
        self.init_scan_state = False  

        self.declare_parameter('front_angle_range', 180) 

        qos = QoSProfile(depth=10)

        # subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        # timer
        self.update_timer = self.create_timer(
            0.100,  
            self.update_callback)

        self.get_logger().info('Arsheida obstacle detection node has been initialised.')

        signal.signal(signal.SIGINT, self.signal_handler)

    # functions
    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    def update_callback(self):
        if self.init_scan_state:
            self.detect_obstacle()

    def detect_obstacle(self):

        self.front_angle_range = self.get_parameter('front_angle_range').get_parameter_value().integer_value        

        total_points = len(self.scan_ranges)
        self.get_logger().info(f'{total_points} : Total data points')
        center_index = total_points // 2
        half_range = self.front_angle_range // 2
        start_index = max(0, center_index - half_range)
        end_index = min(total_points, center_index + half_range)

        front_ranges = [
            r for r in self.scan_ranges[start_index:end_index]
            if r > 0.01 and r < float('inf')
        ]

        if front_ranges:
            obstacle_distance = min(front_ranges)
            self.get_logger().info(
                f"Nearest Obstacle Distance: {obstacle_distance:.2f} m in {self.front_angle_range}° front arc")
        else:
            self.get_logger().info("No valid obstacle data in front arc.")

    def signal_handler(self, signum, frame):
        # Stop the robot before shutting down
        self.get_logger().info("Ctrl+C pressed, stopping the node...")
        rclpy.shutdown()  

def main():
    rclpy.init()
    node = scan_front()  
    rclpy.spin(node)
    rclpy.shutdown()


