import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool, String
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
import time
import json

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # Subscribers
        self.create_subscription(Float32, 'lane_error', self.error_callback, 10)
        self.create_subscription(Bool, 'obstacle_ahead', self.obstacle_callback, 10)
        self.create_subscription(Bool, 'side_clear', self.side_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_teleop', self.teleop_callback, 10)
        self.create_subscription(String, 'traffic_sign', self.sign_callback, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lane_pub = self.create_publisher(Int32, 'target_lane', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # State Machine & Status Variables
        self.state = 1 # 1: L1, 2: Nudge Left, 3: L2, 4: Nudge Right, 5: Obstacle Stop, 6: Return Wait
        self.sign_state = 'none' # 'none', 'stopped', 'slow'
        self.current_sign = 'none'
        self.lane_error = 0.0
        self.obstacle_ahead = False
        self.side_clear = True
        self.teleop_cmd = Twist() # holds latest manual command
        self.last_teleop_time = 0.0
        
        # Declare parameters
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('kd', 0.001)
        self.declare_parameter('base_speed', 0.3)
        self.declare_parameter('nudge_duration', 1.2)
        self.declare_parameter('obstacle_stop_duration', 1.0)
        self.declare_parameter('pass_clearance_duration', 0.5)
        self.declare_parameter('slow_speed', 0.15)
        self.declare_parameter('sign_stop_duration', 10.0)
        self.declare_parameter('enabled', True)
        self.declare_parameter('mode', 'auto')

        # Get initial parameter values
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.nudge_duration = self.get_parameter('nudge_duration').value
        self.obstacle_stop_duration = self.get_parameter('obstacle_stop_duration').value
        self.pass_clearance_duration = self.get_parameter('pass_clearance_duration').value
        self.slow_speed = self.get_parameter('slow_speed').value
        self.sign_stop_duration = self.get_parameter('sign_stop_duration').value
        self.enabled = self.get_parameter('enabled').value
        self.mode = self.get_parameter('mode').value

        self.last_error = 0.0
        self.nudge_start_time = 0.0
        self.stop_start_time = 0.0
        self.clearance_start_time = 0.0
        self.sign_stop_start_time = 0.0
        
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
            elif param.name == 'obstacle_stop_duration':
                self.obstacle_stop_duration = param.value
                self.get_logger().info(f"obstacle_stop_duration updated to: {self.obstacle_stop_duration}")
            elif param.name == 'pass_clearance_duration':
                self.pass_clearance_duration = param.value
                self.get_logger().info(f"pass_clearance_duration updated to: {self.pass_clearance_duration}")
            elif param.name == 'slow_speed':
                self.slow_speed = param.value
                self.get_logger().info(f"slow_speed updated to: {self.slow_speed}")
            elif param.name == 'sign_stop_duration':
                self.sign_stop_duration = param.value
                self.get_logger().info(f"sign_stop_duration updated to: {self.sign_stop_duration}")
            elif param.name == 'enabled':
                self.enabled = param.value
                self.get_logger().info(f"enabled updated to: {self.enabled}")
            elif param.name == 'mode':
                self.mode = param.value
                self.get_logger().info(f"mode updated to: {self.mode}")
        return SetParametersResult(successful=True)

    def error_callback(self, msg): self.lane_error = msg.data
    def obstacle_callback(self, msg): self.obstacle_ahead = msg.data
    def side_callback(self, msg): self.side_clear = msg.data
    def teleop_callback(self, msg):
        self.teleop_cmd = msg
        self.last_teleop_time = time.time()
    def sign_callback(self, msg):
        sign = msg.data
        if sign != self.current_sign:
            self.current_sign = sign
            if sign in ['red', 'pink']:
                self.sign_state = 'stopped'
                self.sign_stop_start_time = time.time()
                self.get_logger().info(f"Traffic sign '{sign}' detected! Stopping for {self.sign_stop_duration}s.")
            elif sign == 'yellow':
                self.sign_state = 'slow'
                self.get_logger().info("Traffic sign 'yellow' detected! Slowing down.")
            elif sign == 'green' or sign == 'none':
                # Only clear 'slow' state, let 'stopped' state complete its timer
                if self.sign_state == 'slow':
                    self.sign_state = 'none'
                    self.get_logger().info("Traffic sign clear/green. Resuming normal speed.")

    def control_loop(self):
        cmd = Twist()
        
        if not self.enabled:
            # E-STOP active: force halt
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.last_error = 0.0
        elif self.mode == 'manual':
            # Teleop/Manual mode: pass-through teleop commands if fresh (< 1s old)
            if time.time() - self.last_teleop_time < 1.0:
                cmd = self.teleop_cmd
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            # Check sign overrides first
            if self.sign_state == 'stopped':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if time.time() - self.sign_stop_start_time > self.sign_stop_duration:
                    self.sign_state = 'none'
                    self.get_logger().info("Sign wait time over. Resuming driving.")
                self.cmd_pub.publish(cmd)
                self.publish_status()
                return

            # Autonomous mode: PID + obstacle avoidance state machine
            if self.sign_state == 'slow':
                cmd.linear.x = self.slow_speed
            else:
                cmd.linear.x = self.base_speed
            
            if self.state == 1: # Follow Lane 1
                cmd.angular.z = self.calculate_pid()
                self.lane_pub.publish(Int32(data=1))
                if self.obstacle_ahead:
                    self.get_logger().info("Obstacle detected! Stopping before avoidance.")
                    self.state = 5 # OBSTACLE_STOP
                    self.stop_start_time = time.time()
            
            elif self.state == 5: # OBSTACLE_STOP
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if time.time() - self.stop_start_time > self.obstacle_stop_duration:
                    self.get_logger().info("Nudging left to avoid obstacle.")
                    self.state = 2 # NUDGE_LEFT
                    self.nudge_start_time = time.time()
                    
            elif self.state == 2: # Nudge Left
                cmd.angular.z = 0.5 # Steer hard left
                if time.time() - self.nudge_start_time > self.nudge_duration:
                    self.get_logger().info("Catching Lane 2.")
                    self.state = 3 # FOLLOW_L2
                    
            elif self.state == 3: # Follow Lane 2
                cmd.angular.z = self.calculate_pid()
                self.lane_pub.publish(Int32(data=2))
                if self.side_clear and not self.obstacle_ahead:
                    self.get_logger().info("Passed obstacle! Waiting clearance before return.")
                    self.state = 6 # RETURN_WAIT
                    self.clearance_start_time = time.time()

            elif self.state == 6: # RETURN_WAIT
                cmd.angular.z = self.calculate_pid()
                self.lane_pub.publish(Int32(data=2))
                if time.time() - self.clearance_start_time > self.pass_clearance_duration:
                    self.get_logger().info("Clearance met, nudging right.")
                    self.state = 4 # NUDGE_RIGHT
                    self.nudge_start_time = time.time()
                    
            elif self.state == 4: # Nudge Right
                cmd.angular.z = -0.5 # Steer hard right
                if time.time() - self.nudge_start_time > self.nudge_duration:
                    self.get_logger().info("Catching Lane 1.")
                    self.state = 1 # FOLLOW_L1

        self.cmd_pub.publish(cmd)
        self.publish_status()

    def publish_status(self):
        # Publish status telemetry as JSON
        status_data = {
            'enabled': self.enabled,
            'mode': self.mode,
            'state': self.state,
            'sign_state': self.sign_state,
            'current_sign': self.current_sign,
            'lane_error': self.lane_error,
            'obstacle_ahead': self.obstacle_ahead,
            'side_clear': self.side_clear,
            'kp': self.kp,
            'kd': self.kd,
            'base_speed': self.base_speed,
            'nudge_duration': self.nudge_duration
        }
        self.status_pub.publish(String(data=json.dumps(status_data)))

    def calculate_pid(self):
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