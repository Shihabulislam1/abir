import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Declare parameters
        self.declare_parameter('video_device', 0)
        self.declare_parameter('target_lane', 1)

        # Get initial parameter values
        self.video_device = self.get_parameter('video_device').value
        self.target_lane = self.get_parameter('target_lane').value

        # Publisher for the error (distance from center)
        self.error_pub = self.create_publisher(Float32, 'lane_error', 10)
        # Subscriber to know which lane we should be looking for
        self.target_lane_sub = self.create_subscription(Int32, 'target_lane', self.lane_callback, 10)
        
        # Open camera
        self.cap = cv2.VideoCapture(self.video_device)
        
        # Register parameters callback
        self.add_on_set_parameters_callback(self.param_callback)

        self.timer = self.create_timer(0.05, self.process_frame) # 20 FPS

    def param_callback(self, params):
        for param in params:
            if param.name == 'target_lane':
                self.target_lane = param.value
                self.get_logger().info(f"target_lane updated to: {self.target_lane}")
            elif param.name == 'video_device':
                self.video_device = param.value
                self.get_logger().info(f"video_device updated to: {self.video_device}. Reopening camera...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.video_device)
        return SetParametersResult(successful=True)

    def lane_callback(self, msg):
        self.target_lane = msg.data

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image")
            return

        # 1. Image Processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # 2. Region of Interest (Bottom half of the screen)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[(0, height), (width, height), (width, height//2), (0, height//2)]])
        cv2.fillPoly(mask, polygon, 255)
        masked_edges = cv2.bitwise_and(edges, mask)

        # 3. Find Lines
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=20)
        
        error = 0.0
        # NOTE: In a full implementation, you would sort lines into left/right and dotted/solid here.
        # For this template, we calculate a simple average center point of detected lines.
        if lines is not None:
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                if x1 < width // 2:
                    left_lines.append(x1)
                else:
                    right_lines.append(x1)
            
            # Simple center calculation based on found lines
            left_x = np.mean(left_lines) if left_lines else 0
            right_x = np.mean(right_lines) if right_lines else width
            lane_center = (left_x + right_x) / 2
            camera_center = width / 2
            
            # Positive error means robot is too far right, negative means too far left
            error = camera_center - lane_center 

        # Publish the error
        msg = Float32()
        msg.data = float(error)
        self.error_pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()