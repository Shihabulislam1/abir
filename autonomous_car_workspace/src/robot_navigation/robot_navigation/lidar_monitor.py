import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
import math

class LidarMonitorNode(Node):
    def __init__(self):
        super().__init__('lidar_monitor_node')
        
        # Declare parameters
        self.declare_parameter('front_danger_zone', 0.6)
        self.declare_parameter('side_safe_zone', 0.8)
        self.declare_parameter('scan_topic', '/scan')

        # Get initial parameter values
        self.front_danger_zone = self.get_parameter('front_danger_zone').value
        self.side_safe_zone = self.get_parameter('side_safe_zone').value
        self.scan_topic = self.get_parameter('scan_topic').value

        # Create subscription and publishers
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_ahead', 10)
        self.side_clear_pub = self.create_publisher(Bool, 'side_clear', 10)

        # Register parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.param_callback)

    def param_callback(self, params):
        for param in params:
            if param.name == 'front_danger_zone':
                self.front_danger_zone = param.value
                self.get_logger().info(f"front_danger_zone updated to: {self.front_danger_zone}")
            elif param.name == 'side_safe_zone':
                self.side_safe_zone = param.value
                self.get_logger().info(f"side_safe_zone updated to: {self.side_safe_zone}")
            elif param.name == 'scan_topic':
                self.scan_topic = param.value
                self.get_logger().info(f"scan_topic updated to: {self.scan_topic}. Re-subscribing...")
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        return SetParametersResult(successful=True)

    def scan_callback(self, msg):
        ranges = msg.ranges
        num_readings = len(ranges)
        
        # Assume 0 degrees is straight ahead. 
        # Check a 30-degree cone in front (-15 to +15 degrees)
        front_indices = list(range(0, 15)) + list(range(num_readings - 15, num_readings))
        obstacle_ahead = False
        for i in front_indices:
            if 0.1 < ranges[i] < self.front_danger_zone: # Ignore 0.0 (error readings)
                obstacle_ahead = True
                break
                
        # Check the right side (approx 260 to 280 degrees assuming 0 is front, counter-clockwise)
        # 270 degrees is exactly to the right.
        side_index_center = int(270 * (num_readings / 360.0))
        side_indices = range(side_index_center - 10, side_index_center + 10)
        side_clear = True
        for i in side_indices:
            if 0.1 < ranges[i] < self.side_safe_zone:
                side_clear = False
                break

        # Publish states
        obs_msg = Bool()
        obs_msg.data = obstacle_ahead
        self.obstacle_pub.publish(obs_msg)

        side_msg = Bool()
        side_msg.data = side_clear
        self.side_clear_pub.publish(side_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()