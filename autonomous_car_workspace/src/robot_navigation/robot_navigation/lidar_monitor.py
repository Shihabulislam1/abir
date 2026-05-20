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
        self.declare_parameter('lidar_offset_deg', 0.0)

        # Get initial parameter values
        self.front_danger_zone = self.get_parameter('front_danger_zone').value
        self.side_safe_zone = self.get_parameter('side_safe_zone').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.lidar_offset_deg = self.get_parameter('lidar_offset_deg').value

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
            elif param.name == 'lidar_offset_deg':
                self.lidar_offset_deg = param.value
                self.get_logger().info(f"lidar_offset_deg updated to: {self.lidar_offset_deg}")
            elif param.name == 'scan_topic':
                self.scan_topic = param.value
                self.get_logger().info(f"scan_topic updated to: {self.scan_topic}. Re-subscribing...")
                self.destroy_subscription(self.scan_sub)
                self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        return SetParametersResult(successful=True)

    def scan_callback(self, msg):
        ranges = msg.ranges
        
        obstacle_ahead = False
        side_clear = True
        
        offset_rad = math.radians(self.lidar_offset_deg)
        
        for i, r in enumerate(ranges):
            # Ignore range values outside spec or empty readings
            if r <= 0.1 or r > msg.range_max or not math.isfinite(r):
                continue
            
            # Calculate actual angle of reading
            angle = msg.angle_min + i * msg.angle_increment + offset_rad
            # Wrap to [-pi, pi]
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            
            # Check front cone: -15 to +15 degrees
            if -math.radians(15) <= angle <= math.radians(15):
                if r < self.front_danger_zone:
                    obstacle_ahead = True
            
            # Check right side: -105 to -75 degrees (-pi/2 +/- 15 deg)
            if -math.radians(105) <= angle <= -math.radians(75):
                if r < self.side_safe_zone:
                    side_clear = False

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