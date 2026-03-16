#!/usr/bin/env python3
"""
Stereo Depth Node for MARY Drone

Computes disparity/depth from the T265 fisheye stereo pair using NVIDIA VPI
on the Jetson Nano GPU. Replaces the standalone realsense_test.py script with
a ROS2 node that subscribes to the realsense2_camera driver topics instead of
using pyrealsense2 directly.

Pipeline:
  1. Subscribe to fisheye1/fisheye2 image_raw + camera_info topics
  2. On first camera_info pair, compute stereo rectification maps (once)
  3. Get extrinsics (R, T) between fisheye frames via tf2
  4. Each frame pair: CPU rectify → VPI GPU stereo → publish disparity + depth
"""

import ctypes
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import message_filters
import tf2_ros


# ---------------------------------------------------------------------------
# VPI stereo backend (wraps libstereo.so compiled from vpi_stereo.cpp)
# ---------------------------------------------------------------------------
class VPISterePipeline:
    """Loads libstereo.so and dispatches rectified frames to VPI/CUDA."""

    def __init__(self, lib_path, width, height, max_disp=256):
        self.input_w = width
        self.input_h = height
        self.output_w = width // 2
        self.output_h = height // 2

        self.lib = ctypes.CDLL(lib_path)
        self.lib.init_pipeline.restype = ctypes.c_void_p
        self.lib.compute_frame.argtypes = [
            ctypes.c_void_p,
            np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS'),
            np.ctypeslib.ndpointer(dtype=np.uint8, flags='C_CONTIGUOUS'),
            np.ctypeslib.ndpointer(dtype=np.uint16, flags='C_CONTIGUOUS'),
        ]

        self.ctx = self.lib.init_pipeline(width, height, max_disp)
        self.d_buffer = np.zeros((self.output_h, self.output_w), dtype=np.uint16)

    def compute(self, left, right):
        self.lib.compute_frame(self.ctx, left, right, self.d_buffer)
        return self.d_buffer


# ---------------------------------------------------------------------------
# ROS2 Node
# ---------------------------------------------------------------------------
class StereoDepthNode(Node):
    """
    Subscriptions:
        /camera/fisheye1/image_raw   (sensor_msgs/Image)
        /camera/fisheye2/image_raw   (sensor_msgs/Image)
        /camera/fisheye1/camera_info (sensor_msgs/CameraInfo)
        /camera/fisheye2/camera_info (sensor_msgs/CameraInfo)

    Publications:
        /mary/depth/disparity  (sensor_msgs/Image, mono16)  — raw Q10.5 disparity
        /mary/depth/depth      (sensor_msgs/Image, 32FC1)   — metric depth in metres
        /mary/depth/debug      (sensor_msgs/Image, bgr8)    — colorised visualisation
    """

    def __init__(self):
        super().__init__('stereo_depth_node')

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter('libstereo_path',
                               os.path.expanduser('~/Documents/SkyNet/ROB498-SkyNet/scripts/libstereo.so'))
        self.declare_parameter('max_disparity', 256)
        self.declare_parameter('zoom_factor', 1.5)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('median_filter_size', 3)   # 0 to disable

        self.lib_path = self.get_parameter('libstereo_path').value
        self.max_disp = self.get_parameter('max_disparity').value
        self.zoom_factor = self.get_parameter('zoom_factor').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.median_ks = self.get_parameter('median_filter_size').value

        # ── State ─────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.pipeline = None          # VPISterePipeline (lazy init)
        self.calibrated = False
        self.l_map1 = None
        self.l_map2 = None
        self.r_map1 = None
        self.r_map2 = None
        self.baseline = None          # metres
        self.focal_length = None      # pixels (in rectified image)

        # Camera info received flags
        self._ci_left = None          # CameraInfo
        self._ci_right = None
        self._extrinsics = None       # (R, T) from tf2

        # ── tf2 ───────────────────────────────────────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Camera info subscribers (latched / transient-local) ───────
        ci_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CameraInfo, '/camera/fisheye1/camera_info',
                                 self._ci_left_cb, ci_qos)
        self.create_subscription(CameraInfo, '/camera/fisheye2/camera_info',
                                 self._ci_right_cb, ci_qos)

        # ── Synchronised image subscribers ────────────────────────────
        sub_left = message_filters.Subscriber(self, Image,
                                              '/camera/fisheye1/image_raw',
                                              qos_profile=sensor_qos)
        sub_right = message_filters.Subscriber(self, Image,
                                               '/camera/fisheye2/image_raw',
                                               qos_profile=sensor_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_left, sub_right], queue_size=5, slop=0.02)
        self.sync.registerCallback(self._stereo_cb)

        # ── Publishers ────────────────────────────────────────────────
        self.pub_disparity = self.create_publisher(Image, '/mary/depth/disparity', 10)
        self.pub_depth = self.create_publisher(Image, '/mary/depth/depth', 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(Image, '/mary/depth/debug', 10)

        self.get_logger().info('Stereo Depth Node initialised — waiting for camera_info + tf')

    # ------------------------------------------------------------------
    # Camera info callbacks
    # ------------------------------------------------------------------
    def _ci_left_cb(self, msg: CameraInfo):
        if self._ci_left is None:
            self._ci_left = msg
            self.get_logger().info(f'Got left camera_info  ({msg.width}x{msg.height})')
            self._try_calibrate()

    def _ci_right_cb(self, msg: CameraInfo):
        if self._ci_right is None:
            self._ci_right = msg
            self.get_logger().info(f'Got right camera_info ({msg.width}x{msg.height})')
            self._try_calibrate()

    # ------------------------------------------------------------------
    # One-shot calibration (mirrors realsense_test.py Phase 2)
    # ------------------------------------------------------------------
    def _try_calibrate(self):
        """Attempt calibration once both camera_info msgs and tf extrinsics are available."""
        if self.calibrated or self._ci_left is None or self._ci_right is None:
            return

        # --- Extrinsics via tf2 ---
        # realsense2_camera publishes static tf between these frames
        left_frame = self._ci_left.header.frame_id
        right_frame = self._ci_right.header.frame_id
        if not left_frame or not right_frame:
            left_frame = 'camera_fisheye1_optical_frame'
            right_frame = 'camera_fisheye2_optical_frame'

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                left_frame, right_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Waiting for tf {left_frame} → {right_frame}: {e}')
            # Retry on next camera_info or via a one-shot timer
            self.create_timer(1.0, self._retry_calibrate)
            return

        self._finish_calibration(tf_msg)

    def _retry_calibrate(self):
        """Timer callback to retry calibration if tf wasn't ready."""
        if self.calibrated:
            return
        self._try_calibrate()

    def _finish_calibration(self, tf_msg):
        """Compute rectification maps from camera_info + tf extrinsics."""
        ci_l, ci_r = self._ci_left, self._ci_right
        w, h = ci_l.width, ci_l.height

        # Intrinsics
        K_left = np.array(ci_l.k).reshape(3, 3)
        K_right = np.array(ci_r.k).reshape(3, 3)
        D_left = np.array(ci_l.d[:4])
        D_right = np.array(ci_r.d[:4])

        # Extrinsics from tf (left → right)
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        T = np.array([t.x, t.y, t.z])

        # Quaternion → rotation matrix (x, y, z, w)
        R = self._quat_to_rotation(q.x, q.y, q.z, q.w)

        self.baseline = np.linalg.norm(T)
        self.get_logger().info(f'Stereo baseline: {self.baseline * 1000:.1f} mm')

        # Stereo rectification
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K_left, D_left, K_right, D_right, (w, h), R, T,
            flags=cv2.fisheye.CALIB_ZERO_DISPARITY,
            balance=0.0, fov_scale=1.0,
        )

        # Custom zoom projection (same as realsense_test.py)
        new_fx = w / 2.0 * self.zoom_factor
        new_fy = h / 2.0 * self.zoom_factor
        P_custom = np.array([
            [new_fx, 0,      w / 2.0, 0],
            [0,      new_fy, h / 2.0, 0],
            [0,      0,      1,       0],
        ])
        self.focal_length = new_fx  # used for depth = baseline * f / disparity

        # Build undistort + rectify maps
        self.l_map1, self.l_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_left, D_left, R1, P_custom, (w, h), cv2.CV_16SC2)
        self.r_map1, self.r_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_right, D_right, R2, P_custom, (w, h), cv2.CV_16SC2)

        # Init VPI pipeline
        try:
            self.pipeline = VPISterePipeline(self.lib_path, w, h, self.max_disp)
        except Exception as e:
            self.get_logger().error(f'Failed to load VPI pipeline: {e}')
            return

        self.calibrated = True
        self.get_logger().info(
            f'Stereo calibration complete — {w}x{h} → {self.pipeline.output_w}x{self.pipeline.output_h} '
            f'(zoom={self.zoom_factor}, max_disp={self.max_disp})'
        )

    # ------------------------------------------------------------------
    # Stereo frame callback
    # ------------------------------------------------------------------
    def _stereo_cb(self, left_msg: Image, right_msg: Image):
        if not self.calibrated:
            return

        # Convert ROS images → grayscale numpy
        left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

        # 1. Rectify (CPU)
        left_rect = cv2.remap(left, self.l_map1, self.l_map2, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.r_map1, self.r_map2, cv2.INTER_LINEAR)
        left_rect = np.ascontiguousarray(left_rect)
        right_rect = np.ascontiguousarray(right_rect)

        # 2. VPI stereo (GPU)
        d_raw = self.pipeline.compute(left_rect, right_rect)

        # 3. Optional median filter
        if self.median_ks > 0:
            d_filtered = cv2.medianBlur(d_raw, self.median_ks)
        else:
            d_filtered = d_raw

        # 4. Publish raw disparity (mono16, Q10.5 fixed-point)
        disp_msg = self.bridge.cv2_to_imgmsg(d_filtered, encoding='mono16')
        disp_msg.header = left_msg.header
        self.pub_disparity.publish(disp_msg)

        # 5. Convert to metric depth and publish
        d_float = d_filtered.astype(np.float32) / 32.0   # Q10.5 → pixel disparity
        depth = np.zeros_like(d_float)
        valid = d_float > 0.5  # avoid div-by-zero
        # depth = baseline * focal_length / disparity
        # focal_length is at full-res; disparity is at half-res → scale focal by 0.5
        depth[valid] = (self.baseline * self.focal_length * 0.5) / d_float[valid]

        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
        depth_msg.header = left_msg.header
        self.pub_depth.publish(depth_msg)

        # 6. Debug visualisation
        if self.publish_debug:
            d_vis = (d_float / 64.0 * 255.0).clip(0, 255).astype(np.uint8)
            d_color = cv2.applyColorMap(d_vis, cv2.COLORMAP_TURBO)
            debug_msg = self.bridge.cv2_to_imgmsg(d_color, encoding='bgr8')
            debug_msg.header = left_msg.header
            self.pub_debug.publish(debug_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_rotation(x, y, z, w):
        """Convert quaternion to 3x3 rotation matrix."""
        # Using direct formula (no tf_transformations dependency)
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        return np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),     2*(xz + wy)],
            [    2*(xy + wz), 1 - 2*(xx + zz),     2*(yz - wx)],
            [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx + yy)],
        ])


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
