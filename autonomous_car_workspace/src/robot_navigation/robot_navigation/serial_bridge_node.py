import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.2)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('speed_to_pwm_factor', 500.0)

        # Get parameter values
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.speed_to_pwm_factor = self.get_parameter('speed_to_pwm_factor').value

        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Connect to Arduino (Check if it's ttyACM0 or ttyUSB0)
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
        except serial.SerialException:
            self.get_logger().error(f"Arduino not found on {self.port}! Check connection.")
            self.ser = None

    def cmd_callback(self, msg):
        if self.ser is None: return

        v = msg.linear.x  # Forward speed
        w = msg.angular.z # Steering speed

        # Differential drive kinematics
        left_speed = v - (w * self.wheel_separation / 2.0)
        right_speed = v + (w * self.wheel_separation / 2.0)

        # Convert raw speed to PWM values (0 to 255)
        left_pwm = int(max(min(left_speed * self.speed_to_pwm_factor, self.max_pwm), -self.max_pwm))
        right_pwm = int(max(min(right_speed * self.speed_to_pwm_factor, self.max_pwm), -self.max_pwm))

        # Format string and send: e.g., "<L:150,R:150>\n"
        command = f"<L:{left_pwm},R:{right_pwm}>\n"
        self.ser.write(command.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()