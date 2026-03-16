#!/usr/bin/env python3
"""
Stereo Tracker Node for MARY Drone

Combines VPI GPU stereo disparity with blob detection to track a person
below the drone using the T265 fisheye stereo pair.  Publishes the
target's 3D position relative to the drone in body frame.

Pipeline:
  1. Subscribe to T265 fisheye1/fisheye2 image + camera_info topics
  2. One-shot stereo calibration from camera_info + tf2 extrinsics
  3. Per frame: CPU rectify -> VPI GPU stereo -> depth threshold -> blob detect
  4. Largest blob centroid -> 3D point (body frame) -> EMA filter -> publish

Subscriptions:
    /camera/fisheye1/image_raw   (sensor_msgs/Image)
    /camera/fisheye2/image_raw   (sensor_msgs/Image)
    /camera/fisheye1/camera_info (sensor_msgs/CameraInfo)
    /camera/fisheye2/camera_info (sensor_msgs/CameraInfo)

Publications:
    /mary/tracking/target  (geometry_msgs/PointStamped) -- 3D offset in body frame
    /mary/tracking/status  (std_msgs/String)            -- TRACKING / LOST
    /mary/tracking/debug   (sensor_msgs/Image)          -- annotated depth image
"""

import ctypes
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

import message_filters


# ---------------------------------------------------------------------------
# VPI stereo backend (wraps libstereo.so compiled from vpi_stereo.cpp)
# ---------------------------------------------------------------------------
class VPIStereoPipeline:
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
class StereoTrackerNode(Node):

    def __init__(self):
        super().__init__('stereo_tracker_node')

        # -- Parameters -------------------------------------------------------
        self.declare_parameter(
            'libstereo_path',
            os.path.expanduser(
                '~/Documents/SkyNet/ROB498-SkyNet/scripts/libstereo.so'))
        self.declare_parameter('max_disparity', 256)
        self.declare_parameter('zoom_factor', 1.5)
        self.declare_parameter('median_filter_size', 3)
        self.declare_parameter('publish_debug', True)

        # Blob detection
        self.declare_parameter('depth_min', 0.5)        # m -- closest valid depth
        self.declare_parameter('depth_max', 2.5)        # m -- farthest valid depth
        self.declare_parameter('min_blob_area', 500)    # px at half-res
        self.declare_parameter('max_blob_count', 5)

        # Tracking filter
        self.declare_parameter('ema_alpha', 0.3)
        self.declare_parameter('lost_threshold', 15)    # frames -> LOST

        # Axis mapping: camera image -> body frame.
        # Signs depend on T265 mounting and may need flipping during testing.
        # Default assumes T265 bottom-facing, lenses backward:
        #   image +v (down in image, away from lenses) -> body +X (forward)
        #   image +u (right in image)                  -> body +Y (right)
        #   depth  (into image = below drone)          -> body -Z (down)
        self.declare_parameter('axis_sign_x', 1.0)
        self.declare_parameter('axis_sign_y', 1.0)

        self.lib_path       = self.get_parameter('libstereo_path').value
        self.max_disp       = self.get_parameter('max_disparity').value
        self.zoom_factor    = self.get_parameter('zoom_factor').value
        self.median_ks      = self.get_parameter('median_filter_size').value
        self.publish_debug  = self.get_parameter('publish_debug').value

        self.depth_min      = self.get_parameter('depth_min').value
        self.depth_max      = self.get_parameter('depth_max').value
        self.min_blob_area  = self.get_parameter('min_blob_area').value
        self.max_blob_count = self.get_parameter('max_blob_count').value

        self.ema_alpha      = self.get_parameter('ema_alpha').value
        self.lost_threshold = self.get_parameter('lost_threshold').value

        self.axis_sign_x    = self.get_parameter('axis_sign_x').value
        self.axis_sign_y    = self.get_parameter('axis_sign_y').value

        # -- State ------------------------------------------------------------
        self.bridge    = CvBridge()
        self.pipeline  = None          # VPIStereoPipeline (lazy init)
        self.calibrated = False
        self.l_map1 = self.l_map2 = None
        self.r_map1 = self.r_map2 = None
        self.baseline     = None       # metres
        self.focal_length = None       # px at full res

        self._ci_left  = None          # CameraInfo
        self._ci_right = None

        # Rectified intrinsics at half resolution (VPI output size)
        self.rect_fx = None
        self.rect_fy = None
        self.rect_cx = None
        self.rect_cy = None

        # Tracking state
        self.tracked_point = None      # filtered [x, y, z] body frame
        self.frames_without_detection = 0
        self.tracking_status = 'LOST'

        # -- QoS --------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        ci_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -- Camera info subscribers ------------------------------------------
        self.create_subscription(
            CameraInfo, '/camera/fisheye1/camera_info',
            self._ci_left_cb, ci_qos)
        self.create_subscription(
            CameraInfo, '/camera/fisheye2/camera_info',
            self._ci_right_cb, ci_qos)

        # -- Synchronised image subscribers -----------------------------------
        sub_left = message_filters.Subscriber(
            self, Image, '/camera/fisheye1/image_raw', qos_profile=sensor_qos)
        sub_right = message_filters.Subscriber(
            self, Image, '/camera/fisheye2/image_raw', qos_profile=sensor_qos)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_left, sub_right], queue_size=5, slop=0.02)
        self.sync.registerCallback(self._stereo_cb)

        # -- Publishers -------------------------------------------------------
        self.pub_target = self.create_publisher(
            PointStamped, '/mary/tracking/target', 10)
        self.pub_status = self.create_publisher(
            String, '/mary/tracking/status', 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/mary/tracking/debug', 10)

        self.get_logger().info(
            'StereoTrackerNode initialised -- waiting for calibration')

    # ------------------------------------------------------------------
    # Camera info + calibration
    # ------------------------------------------------------------------
    def _ci_left_cb(self, msg):
        if self._ci_left is None:
            self._ci_left = msg
            self.get_logger().info(
                f'Got left camera_info ({msg.width}x{msg.height})')
            self._try_calibrate()

    def _ci_right_cb(self, msg):
        if self._ci_right is None:
            self._ci_right = msg
            self.get_logger().info(
                f'Got right camera_info ({msg.width}x{msg.height})')
            self._try_calibrate()

    def _try_calibrate(self):
        if self.calibrated or self._ci_left is None or self._ci_right is None:
            return
        self._finish_calibration()

    def _finish_calibration(self):
        ci_l, ci_r = self._ci_left, self._ci_right
        w, h = ci_l.width, ci_l.height

        K_left  = np.array(ci_l.k).reshape(3, 3)
        K_right = np.array(ci_r.k).reshape(3, 3)
        D_left  = np.array(ci_l.d[:4])
        D_right = np.array(ci_r.d[:4])

        # Extrinsics: T265 factory-calibrated values from pyrealsense2 EEPROM.
        # (printed by scripts/print_calibration.py — matches realsense_test.py)
        T = np.array([-0.06430789083242416, 7.151146564865485e-05, -0.00014938200183678418])
        R = np.array([
            [ 0.9999712705612183,  0.0010891810525208712, -0.007505698595196009],
            [-0.0010844767093658447, 0.9999992251396179,   0.0006306868162937462],
            [ 0.007506378460675478, -0.0006225303513929248, 0.9999716281890869],
        ])

        self.baseline = np.linalg.norm(T)
        self.get_logger().info(
            f'Stereo baseline: {self.baseline * 1000:.1f} mm (T265 factory)')

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
        self.focal_length = new_fx

        self.l_map1, self.l_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_left, D_left, R1, P_custom, (w, h), cv2.CV_16SC2)
        self.r_map1, self.r_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_right, D_right, R2, P_custom, (w, h), cv2.CV_16SC2)

        # Intrinsics at half resolution (VPI output)
        self.rect_fx = new_fx * 0.5
        self.rect_fy = new_fy * 0.5
        self.rect_cx = (w / 2.0) * 0.5
        self.rect_cy = (h / 2.0) * 0.5

        try:
            self.pipeline = VPIStereoPipeline(
                self.lib_path, w, h, self.max_disp)
        except Exception as e:
            self.get_logger().error(f'Failed to load VPI pipeline: {e}')
            return

        self.calibrated = True
        self.get_logger().info(
            f'Calibration complete -- {w}x{h} -> '
            f'{self.pipeline.output_w}x{self.pipeline.output_h}  '
            f'baseline={self.baseline*1000:.1f}mm  f={self.focal_length:.1f}px')

    # ------------------------------------------------------------------
    # Stereo frame callback
    # ------------------------------------------------------------------
    def _stereo_cb(self, left_msg, right_msg):
        if not self.calibrated:
            return

        left  = self.bridge.imgmsg_to_cv2(left_msg,  desired_encoding='mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

        # 1. Rectify (CPU)
        left_rect  = cv2.remap(left,  self.l_map1, self.l_map2, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.r_map1, self.r_map2, cv2.INTER_LINEAR)
        left_rect  = np.ascontiguousarray(left_rect)
        right_rect = np.ascontiguousarray(right_rect)

        # 2. VPI stereo (GPU)
        d_raw = self.pipeline.compute(left_rect, right_rect)

        # 3. Optional median filter
        if self.median_ks > 0:
            d_raw = cv2.medianBlur(d_raw, self.median_ks)

        # 4. Disparity -> depth
        d_float = d_raw.astype(np.float32) / 32.0  # Q10.5 -> pixel disparity
        depth = np.zeros_like(d_float)
        valid = d_float > 0.5
        depth[valid] = (self.baseline * self.rect_fx) / d_float[valid]

        # 5. Blob detection on depth band
        detection = self._detect_blob(depth)

        # 6. Update tracking state
        if detection is not None:
            cx, cy, blob_depth = detection

            # Pixel + depth -> 3D offset in camera optical frame
            x_cam = (cx - self.rect_cx) / self.rect_fx * blob_depth
            y_cam = (cy - self.rect_cy) / self.rect_fy * blob_depth

            # Map to body frame (configurable signs for mounting)
            # Camera looks down: image v -> body X (forward),
            #                    image u -> body Y (right),
            #                    depth   -> body -Z (down)
            x_body = self.axis_sign_x * y_cam
            y_body = self.axis_sign_y * x_cam
            z_body = -blob_depth

            raw_point = np.array([x_body, y_body, z_body])

            # EMA filter
            if self.tracked_point is None:
                self.tracked_point = raw_point
            else:
                self.tracked_point = (
                    self.ema_alpha * raw_point
                    + (1.0 - self.ema_alpha) * self.tracked_point)

            self.frames_without_detection = 0
            self.tracking_status = 'TRACKING'
        else:
            self.frames_without_detection += 1
            if self.frames_without_detection >= self.lost_threshold:
                self.tracking_status = 'LOST'
                self.tracked_point = None

        # 7. Publish status
        status_msg = String()
        status_msg.data = self.tracking_status
        self.pub_status.publish(status_msg)

        # 8. Publish target if tracking
        if self.tracked_point is not None:
            target_msg = PointStamped()
            target_msg.header = left_msg.header
            target_msg.header.frame_id = 'base_link'
            target_msg.point.x = float(self.tracked_point[0])
            target_msg.point.y = float(self.tracked_point[1])
            target_msg.point.z = float(self.tracked_point[2])
            self.pub_target.publish(target_msg)

        # 9. Debug visualisation
        if self.publish_debug:
            self._publish_debug(depth, detection, left_msg.header)

    # ------------------------------------------------------------------
    # Blob detection
    # ------------------------------------------------------------------
    def _detect_blob(self, depth):
        """Find the largest depth blob in the valid range.

        Returns (cx, cy, mean_depth) or None.
        """
        # Binary mask: pixels within depth band
        mask = ((depth > self.depth_min)
                & (depth < self.depth_max)).astype(np.uint8) * 255

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Sort by area descending, check top candidates
        contours = sorted(
            contours, key=cv2.contourArea, reverse=True)[:self.max_blob_count]

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_blob_area:
                continue

            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']

            # Mean depth within blob
            blob_mask = np.zeros(depth.shape, dtype=np.uint8)
            cv2.drawContours(blob_mask, [contour], -1, 255, -1)
            blob_depths = depth[blob_mask > 0]
            valid = blob_depths[
                (blob_depths > self.depth_min)
                & (blob_depths < self.depth_max)]
            if len(valid) == 0:
                continue

            return (cx, cy, float(np.median(valid)))

        return None

    # ------------------------------------------------------------------
    # Debug visualisation
    # ------------------------------------------------------------------
    def _publish_debug(self, depth, detection, header):
        d_vis = np.zeros(depth.shape, dtype=np.uint8)
        valid = depth > 0
        if np.any(valid):
            d_norm = depth.copy()
            d_norm[~valid] = 0
            d_vis = (d_norm / self.depth_max * 255.0).clip(0, 255).astype(
                np.uint8)

        d_color = cv2.applyColorMap(d_vis, cv2.COLORMAP_TURBO)

        # Draw detection marker
        if detection is not None:
            cx, cy, d = detection
            cv2.circle(d_color, (int(cx), int(cy)), 10, (0, 255, 0), 2)
            cv2.putText(
                d_color, f'{d:.2f}m', (int(cx) + 15, int(cy)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Status overlay
        color = (0, 255, 0) if self.tracking_status == 'TRACKING' \
            else (0, 0, 255)
        cv2.putText(
            d_color, self.tracking_status, (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        debug_msg = self.bridge.cv2_to_imgmsg(d_color, encoding='bgr8')
        debug_msg.header = header
        self.pub_debug.publish(debug_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _quat_to_rotation(x, y, z, w):
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
    node = StereoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
