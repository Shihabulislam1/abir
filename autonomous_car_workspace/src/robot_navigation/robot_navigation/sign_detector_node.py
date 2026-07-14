import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector_node')
        
        self.declare_parameter('min_sign_area', 500)
        self.declare_parameter('circularity_threshold', 0.7)
        self.declare_parameter('detection_confidence_frames', 3)
        self.declare_parameter('sign_cooldown', 12.0)
        
        self.declare_parameter('red_h_low', 0)
        self.declare_parameter('red_h_high', 10)
        self.declare_parameter('red2_h_low', 160)
        self.declare_parameter('red2_h_high', 180)
        self.declare_parameter('pink_h_low', 140)
        self.declare_parameter('pink_h_high', 170)
        self.declare_parameter('yellow_h_low', 20)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('green_h_low', 40)
        self.declare_parameter('green_h_high', 85)
        
        self.declare_parameter('s_low', 100)
        self.declare_parameter('v_low', 80)
        
        self.min_sign_area = self.get_parameter('min_sign_area').value
        self.circularity_threshold = self.get_parameter('circularity_threshold').value
        self.detection_confidence_frames = self.get_parameter('detection_confidence_frames').value
        self.sign_cooldown = self.get_parameter('sign_cooldown').value
        
        self.bridge = CvBridge()
        
        # Subscribe to camera/debug_image
        self.image_sub = self.create_subscription(
            Image,
            'camera/debug_image',
            self.image_callback,
            10
        )
        
        # Publish traffic sign
        self.sign_pub = self.create_publisher(String, 'traffic_sign', 10)
        
        self.consecutive_detections = {'red': 0, 'pink': 0, 'yellow': 0, 'green': 0, 'none': 0}
        self.last_sign = 'none'
        self.last_sign_time = 0.0

    def get_hsv_ranges(self):
        s_low = self.get_parameter('s_low').value
        v_low = self.get_parameter('v_low').value
        return {
            'red': [
                (np.array([self.get_parameter('red_h_low').value, s_low, v_low]), 
                 np.array([self.get_parameter('red_h_high').value, 255, 255])),
                (np.array([self.get_parameter('red2_h_low').value, s_low, v_low]), 
                 np.array([self.get_parameter('red2_h_high').value, 255, 255]))
            ],
            'pink': [
                (np.array([self.get_parameter('pink_h_low').value, 50, v_low]), # slightly lower S for pink
                 np.array([self.get_parameter('pink_h_high').value, 255, 255]))
            ],
            'yellow': [
                (np.array([self.get_parameter('yellow_h_low').value, s_low, v_low]), 
                 np.array([self.get_parameter('yellow_h_high').value, 255, 255]))
            ],
            'green': [
                (np.array([self.get_parameter('green_h_low').value, 80, v_low]), # slightly lower S for green
                 np.array([self.get_parameter('green_h_high').value, 255, 255]))
            ]
        }

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return
            
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        ranges_dict = self.get_hsv_ranges()
        
        detected_sign = 'none'
        
        # Check cooldown
        if time.time() - self.last_sign_time < self.sign_cooldown and self.last_sign != 'none':
             pass # In cooldown, don't detect new signs
        else:
            for color, color_ranges in ranges_dict.items():
                mask = None
                for (lower, upper) in color_ranges:
                    if mask is None:
                        mask = cv2.inRange(hsv_image, lower, upper)
                    else:
                        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))
                        
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < self.min_sign_area:
                        continue
                        
                    (x, y), radius = cv2.minEnclosingCircle(cnt)
                    circle_area = math.pi * (radius ** 2)
                    
                    if circle_area == 0:
                        continue
                        
                    circularity = area / circle_area
                    if circularity > self.circularity_threshold:
                        detected_sign = color
                        break # Found one sign, stop checking contours for this color
                if detected_sign != 'none':
                    break # Stop checking other colors
                    
        # Update consecutive detections
        for color in self.consecutive_detections:
            if color == detected_sign:
                self.consecutive_detections[color] += 1
            else:
                self.consecutive_detections[color] = 0
                
        # Confirm detection
        confirmed_sign = 'none'
        for color, count in self.consecutive_detections.items():
            if count >= self.detection_confidence_frames:
                confirmed_sign = color
                break
                
        # If we confirm a sign, and it's different from last or cooldown passed
        if confirmed_sign != 'none' and (confirmed_sign != self.last_sign or time.time() - self.last_sign_time > self.sign_cooldown):
            self.last_sign = confirmed_sign
            self.last_sign_time = time.time()
            self.get_logger().info(f"Confirmed sign: {confirmed_sign}")
            
        pub_msg = String()
        pub_msg.data = confirmed_sign
        self.sign_pub.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
