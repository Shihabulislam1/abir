import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import time

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # Subscribers
        self.create_subscription(Float32, 'lane_error', self.error_callback, 10)
        self.create_subscription(Bool, 'obstacle_ahead', self.obstacle_callback, 10)
        self.create_subscription(Bool, 'side_clear', self.side_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lane_pub = self.create_publisher(Int32, 'target_lane', 10)
        
        # State Machine Variables
        self.state = 1 # 1: L1, 2: Nudge Left, 3: L2, 4: Nudge Right
        self.lane_error = 0.0
        self.obstacle_ahead = False
        self.side_clear = True
        
        # Declare parameters
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('base_speed', 0.3)
        self.declare_parameter('nudge_duration', 1.2)

        # Get initial parameter values
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.nudge_duration = self.get_parameter('nudge_duration').value

        self.last_error = 0.0
        self.nudge_start_time = 0.0
        
        # Register callback for dynamic parameter updates
        self.add_on_set_parameters_callback(self.param_callback)

        self.control_timer = self.create_timer(0.05, self.control_loop) # 20 Hz

    def param_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f"kp updated to: {self.kp}")
            elif param.name == 'kd':
                self.kd = param.value
                self.get_logger().info(f"kd updated to: {self.kd}")
            elif param.name == 'base_speed':
                self.base_speed = param.value
                self.get_logger().info(f"base_speed updated to: {self.base_speed}")
            elif param.name == 'nudge_duration':
                self.nudge_duration = param.value
                self.get_logger().info(f"nudge_duration updated to: {self.nudge_duration}")
        return SetParametersResult(successful=True)

    def error_callback(self, msg): self.lane_error = msg.data
    def obstacle_callback(self, msg): self.obstacle_ahead = msg.data
    def side_callback(self, msg): self.side_clear = msg.data

    def control_loop(self):
        cmd = Twist()
        cmd.linear.x = self.base_speed
        
        if self.state == 1: # Follow Lane 1
            cmd.angular.z = self.calculate_pid()
            self.lane_pub.publish(Int32(data=1))
            if self.obstacle_ahead:
                self.get_logger().info("Obstacle detected! Nudging left.")
                self.state = 2
                self.nudge_start_time = time.time()
                
        elif self.state == 2: # Nudge Left
            cmd.angular.z = 0.5 # Steer hard left
            if time.time() - self.nudge_start_time > self.nudge_duration:
                self.get_logger().info("Catching Lane 2.")
                self.state = 3
                
        elif self.state == 3: # Follow Lane 2
            cmd.angular.z = self.calculate_pid()
            self.lane_pub.publish(Int32(data=2))
            if self.side_clear and not self.obstacle_ahead:
                self.get_logger().info("Passed obstacle! Nudging right.")
                self.state = 4
                self.nudge_start_time = time.time()
                
        elif self.state == 4: # Nudge Right
            cmd.angular.z = -0.5 # Steer hard right
            if time.time() - self.nudge_start_time > self.nudge_duration:
                self.get_logger().info("Catching Lane 1.")
                self.state = 1

        self.cmd_pub.publish(cmd)

    def calculate_pid(self):
        # Proportional-Derivative control for smooth steering
        p = self.kp * self.lane_error
        d = self.kd * (self.lane_error - self.last_error)
        self.last_error = self.lane_error
        return float(p + d)

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()