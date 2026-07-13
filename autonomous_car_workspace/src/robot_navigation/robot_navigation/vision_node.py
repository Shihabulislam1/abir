import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
import cv2
import numpy as np
import time
import math
from typing import Optional

from .vision import LaneDetector, LaneDetection, draw_overlay
from .camera import WebcamFrameSource, PiCameraFrameSource, AnnotatedFrameProvider, MJPEGStreamer
from .camera.streamer import TunerHooks


class VisionConfig:
    """Wraps ROS2 parameters as attribute access for the LaneDetector."""
    def __init__(self, node: Node):
        self.node = node
        
        # We define a mapping of parameter names to defaults here so we can 
        # iterate and expose them all to the TunerHooks
        self._params = {
            'persp_top_y_frac': 0.18,
            'persp_bottom_y_frac': 0.88,
            'persp_top_left_frac': 0.15,
            'persp_top_right_frac': 0.72,
            'persp_bottom_left_frac': -0.05,
            'persp_bottom_right_frac': 1.00,
            'persp_dst_margin_frac': 0.10,
            'car_center_x_frac': 0.50,
            'adaptive_block_size': 31,
            'adaptive_c': 10,
            'dilate_kernel': 3,
            'sheet_threshold': 110,
            'sheet_close_kernel': 25,
            'sheet_erode_kernel': 9,
            'clahe_enabled': True,
            'clahe_clip': 2.5,
            'clahe_tile': 8,
            'open_kernel': 3,
            'piece_min_area': 20,
            'cluster_gap_px_bird': 55,
            'min_lane_height_frac_bird': 0.30,
            'min_lane_points': 120,
            'near_car_fit_frac': 0.60,
            'lane_half_width_bird': 100.0,
            'look_ahead_frac': 0.6,
            'fit_degree': 2,
            'nwindows': 12,
            'window_margin': 55,
            'window_minpix': 40,
            'min_peak_separation': 90,
        }
        for name, default in self._params.items():
            if not node.has_parameter(name):
                node.declare_parameter(name, default)

    def __getattr__(self, name):
        try:
            return self.node.get_parameter(name).value
        except Exception:
            raise AttributeError(f"VisionConfig has no attribute {name}")


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # Parameters
        self.declare_parameter('video_device', 1)
        self.declare_parameter('target_lane', 1)
        self.declare_parameter('camera_source', 'webcam')  # NEW: 'webcam' or 'picamera'
        self.declare_parameter('camera_rotate_180', False)  # NEW
        self.declare_parameter('stream_enabled', True)      # NEW
        self.declare_parameter('stream_port', 8080)         # NEW
        self.declare_parameter('jpeg_quality', 80)          # NEW
        self.declare_parameter('lidar_overlay_enabled', True)  # NEW
        self.declare_parameter('lidar_max_range_mm', 6000.0)   # NEW
        self.declare_parameter('lidar_inset_size_px', 220)     # NEW
        
        self.target_lane = self.get_parameter('target_lane').value

        # Vision config + detector
        self.cfg = VisionConfig(self)
        self.detector = LaneDetector(self.cfg)

        # Camera (threaded capture)
        self._setup_camera()

        # Annotated frame provider (for streamer)
        self.viz = AnnotatedFrameProvider(self.source)

        # MJPEG streamer
        self._setup_streamer()

        # LiDAR subscription for overlay
        self._lidar_scan = None
        if self.get_parameter('lidar_overlay_enabled').value:
            self.lidar_sub = self.create_subscription(
                LaserScan, '/scan', self._lidar_callback, 10)

        # Publishers (same as before)
        self.error_pub = self.create_publisher(Float32, 'lane_error', 10)
        self.image_pub = self.create_publisher(Image, 'camera/debug_image', 10)
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, 'camera/debug_image/compressed', 10)
        self.bridge = CvBridge()

        # Subscriber
        self.target_lane_sub = self.create_subscription(
            Int32, 'target_lane', self.lane_callback, 10)

        self.add_on_set_parameters_callback(self.param_callback)
        self.timer = self.create_timer(0.05, self.process_frame)

    def _setup_camera(self):
        """Create the appropriate FrameSource based on parameters."""
        source_type = self.get_parameter('camera_source').value
        device = self.get_parameter('video_device').value
        rotate = self.get_parameter('camera_rotate_180').value

        if source_type == 'picamera':
            self.source = PiCameraFrameSource(
                width=640, height=480, framerate=30, rotate_180=rotate)
        else:
            self.source = WebcamFrameSource(
                webcam_index=device, width=640, height=480,
                framerate=30, rotate_180=rotate)
        self.source.start()

    def _setup_streamer(self):
        """Launch MJPEG HTTP streamer if enabled."""
        if self.get_parameter('stream_enabled').value:
            tunable = list(self.cfg._params.keys())  # all vision params
            hooks = TunerHooks(self, tunable)
            port = self.get_parameter('stream_port').value
            quality = self.get_parameter('jpeg_quality').value
            self.streamer = MJPEGStreamer(
                self.viz, port=port, jpeg_quality=quality, tuner_hooks=hooks)
            self.streamer.start()
            self.get_logger().info(f"MJPEG streamer on http://0.0.0.0:{port}/")
        else:
            self.streamer = None

    def _lidar_callback(self, msg):
        """Convert LaserScan → Nx2 (angle_deg, distance_mm) for overlay."""
        n = len(msg.ranges)
        angles = np.array([
            math.degrees(msg.angle_min + i * msg.angle_increment)
            for i in range(n)
        ], dtype=np.float32)
        dists = np.array(msg.ranges, dtype=np.float32) * 1000.0  # m → mm
        valid = np.isfinite(dists) & (dists > 0)
        self._lidar_scan = np.column_stack([angles[valid], dists[valid]])

    def param_callback(self, params):
        for param in params:
            if param.name == 'target_lane':
                self.target_lane = param.value
                self.get_logger().info(f"target_lane updated to: {self.target_lane}")
            elif param.name == 'video_device' or param.name == 'camera_source':
                self.get_logger().info(f"Camera parameters updated. Restarting camera...")
                self.source.stop()
                self._setup_camera()
                # Update the viz provider with the new source
                self.viz.source = self.source
        return SetParametersResult(successful=True)

    def lane_callback(self, msg):
        self.target_lane = msg.data

    def process_frame(self):
        t0 = time.time()
        frame = self.source.get_frame()
        if frame is None:
            return

        det = self.detector.detect(frame)
        target_lane_str = "L" if self.target_lane == 2 else "R"

        # Compute error
        error = 0.0
        if det.found:
            offset = det.lane_center_offset(target_lane_str)
            if offset is not None:
                error = -offset * det.bird_size[0] * 0.5
        self.error_pub.publish(Float32(data=float(error)))

        # Draw overlay with LiDAR inset
        steering = -det.lane_center_offset(target_lane_str) if det.found else None
        fps = 1.0 / (time.time() - t0 + 1e-6)
        debug_frame = draw_overlay(
            frame, det,
            steering=steering, fps=fps,
            target_lane=target_lane_str,
            perspective=self.detector._persp,
            lidar_scan=self._lidar_scan,
            lidar_max_range_mm=self.get_parameter('lidar_max_range_mm').value,
            lidar_inset_size_px=self.get_parameter('lidar_inset_size_px').value,
        )

        # Push to annotated frame provider (for streamer)
        self.viz.set_annotated(debug_frame)

        # Publish ROS2 image topics
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_frame, "bgr8"))
        ok, jpg = cv2.imencode('.jpg', debug_frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if ok:
            comp = CompressedImage()
            comp.header.stamp = self.get_clock().now().to_msg()
            comp.header.frame_id = "camera_link"
            comp.format = "jpeg"
            comp.data = jpg.tobytes()
            self.compressed_image_pub.publish(comp)

    def destroy_node(self):
        self.source.stop()
        if self.streamer:
            self.streamer.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()