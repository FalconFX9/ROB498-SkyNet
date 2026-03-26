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
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
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
        self.declare_parameter('max_disparity', 64)
        self.declare_parameter('zoom_factor', 1.5)
        self.declare_parameter('median_filter_size', 3)
        self.declare_parameter('publish_debug', True)

        # Blob detection
        self.declare_parameter('depth_min', 0.5)        # m -- closest valid depth
        self.declare_parameter('depth_max', 3.5)        # m -- farthest valid depth
        self.declare_parameter('min_blob_area', 125)    # px at half-res
        self.declare_parameter('max_blob_count', 5)

        # Foreground detection
        self.declare_parameter('height_threshold', 0.15) # m above ground to count as foreground

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

        # Video recording
        self.declare_parameter('record_video', False)
        self.declare_parameter('video_dir',
                               os.path.expanduser(
                                   '~/Documents/SkyNet/ROB498-SkyNet/logs'))

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

        self.record_video   = self.get_parameter('record_video').value
        self.video_dir      = self.get_parameter('video_dir').value

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

        # External state for debug overlay
        self._flight_state = 'N/A'
        self._current_pose = None      # PoseStamped
        self._current_setpoint = None  # PoseStamped
        self._actual_fps = 0.0
        self._frame_count = 0
        self._fps_timer = None

        # Video recording (lazy init on first debug frame)
        self._vid_writer = None        # cv2.VideoWriter
        self._vid_path = None
        if self.record_video:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            os.makedirs(self.video_dir, exist_ok=True)
            self._vid_path = os.path.join(self.video_dir, f'tracking_{ts}.mkv')
            self._prune_old_videos(keep=2)  # 2 old + 1 new = 3 total
            self.get_logger().info(f'Video recording enabled -> {self._vid_path}')

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
        # T265 hardware-syncs both fisheye frames with identical stamps,
        # so use exact TimeSynchronizer for O(1) matching (no slop search).
        self.sync = message_filters.TimeSynchronizer(
            [sub_left, sub_right], queue_size=5)
        self.sync.registerCallback(self._stereo_cb)

        # -- Publishers -------------------------------------------------------
        self.pub_target = self.create_publisher(
            PointStamped, '/mary/tracking/target', 10)
        self.pub_status = self.create_publisher(
            String, '/mary/tracking/status', 10)
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                Image, '/mary/tracking/debug', 10)

        # -- Subscribers for debug overlay ------------------------------------
        self.create_subscription(
            String, '/mary/comm/flight_state',
            lambda m: setattr(self, '_flight_state', m.data), 10)
        self.create_subscription(
            PoseStamped, '/mary/localization/pose',
            lambda m: setattr(self, '_current_pose', m), 10)
        self.create_subscription(
            PoseStamped, '/mavros/setpoint_position/local',
            lambda m: setattr(self, '_current_setpoint', m), 10)

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
        w_full, h_full = ci_l.width, ci_l.height

        # Half-res dimensions for CPU remap (optimization: 4x less remap work)
        w_half = w_full // 2
        h_half = h_full // 2
        self._full_w = w_full
        self._full_h = h_full
        self._half_w = w_half
        self._half_h = h_half

        # Use full-res intrinsics + distortion so initUndistortRectifyMap
        # handles both undistortion AND downsampling in a single remap pass.
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

        # Rectify using full-res K for stereoRectify (defines the geometry)
        # but we scale K by 0.5 only for stereoRectify's size parameter
        K_left_half  = K_left.copy();  K_left_half[0, :] *= 0.5;  K_left_half[1, :] *= 0.5
        K_right_half = K_right.copy(); K_right_half[0, :] *= 0.5; K_right_half[1, :] *= 0.5
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K_left_half, D_left, K_right_half, D_right,
            (w_half, h_half), R, T,
            flags=cv2.fisheye.CALIB_ZERO_DISPARITY,
            balance=0.0, fov_scale=1.0,
        )

        # Custom zoom projection at half resolution
        new_fx = w_half / 2.0 * self.zoom_factor
        new_fy = h_half / 2.0 * self.zoom_factor
        P_custom = np.array([
            [new_fx, 0,      w_half / 2.0, 0],
            [0,      new_fy, h_half / 2.0, 0],
            [0,      0,      1,            0],
        ])
        self.focal_length = new_fx

        # Build remap tables: full-res K/D -> half-res output in one pass.
        # This fuses undistortion + downsampling, eliminating cv2.resize.
        self.l_map1, self.l_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_left, D_left, R1, P_custom, (w_half, h_half), cv2.CV_16SC2)
        self.r_map1, self.r_map2 = cv2.fisheye.initUndistortRectifyMap(
            K_right, D_right, R2, P_custom, (w_half, h_half), cv2.CV_16SC2)

        # Intrinsics at VPI output resolution (half of half = quarter)
        self.rect_fx = new_fx * 0.5
        self.rect_fy = new_fy * 0.5
        self.rect_cx = (w_half / 2.0) * 0.5
        self.rect_cy = (h_half / 2.0) * 0.5

        # Pre-allocate reusable buffers to avoid per-frame GC pressure
        self._left_rect_buf  = np.empty((h_half, w_half), dtype=np.uint8)
        self._right_rect_buf = np.empty((h_half, w_half), dtype=np.uint8)
        vpi_out_h, vpi_out_w = h_half // 2, w_half // 2
        self._d_float_buf = np.empty((vpi_out_h, vpi_out_w), dtype=np.float32)
        self._depth_buf   = np.zeros((vpi_out_h, vpi_out_w), dtype=np.float32)

        try:
            # VPI takes half-res as "full", outputs quarter-res
            self.pipeline = VPIStereoPipeline(
                self.lib_path, w_half, h_half, self.max_disp)
        except Exception as e:
            self.get_logger().error(f'Failed to load VPI pipeline: {e}')
            return

        self.calibrated = True
        self.get_logger().info(
            f'Calibration complete -- {w_full}x{h_full} -> '
            f'{w_half}x{h_half} (remap) -> '
            f'{self.pipeline.output_w}x{self.pipeline.output_h} (VPI)  '
            f'baseline={self.baseline*1000:.1f}mm  f={self.focal_length:.1f}px')

    # ------------------------------------------------------------------
    # Stereo frame callback
    # ------------------------------------------------------------------
    def _stereo_cb(self, left_msg, right_msg):
        if not self.calibrated:
            return

        left  = self.bridge.imgmsg_to_cv2(left_msg,  desired_encoding='mono8')
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

        # 1+2. Fused undistort + downsample + rectify in one remap pass (CPU).
        #       Maps were built with full-res K/D -> half-res output.
        cv2.remap(left,  self.l_map1, self.l_map2, cv2.INTER_LINEAR,
                  dst=self._left_rect_buf)
        cv2.remap(right, self.r_map1, self.r_map2, cv2.INTER_LINEAR,
                  dst=self._right_rect_buf)
        left_rect  = self._left_rect_buf
        right_rect = self._right_rect_buf

        # 2. VPI stereo (GPU)
        d_raw = self.pipeline.compute(left_rect, right_rect)

        # 3. Optional median filter
        if self.median_ks > 0:
            d_raw = cv2.medianBlur(d_raw, self.median_ks)

        # 4. Disparity -> depth (in-place, no allocation)
        np.copyto(self._d_float_buf, d_raw, casting='unsafe')
        self._d_float_buf *= (1.0 / 32.0)  # Q10.5 -> pixel disparity
        d_float = self._d_float_buf
        depth = self._depth_buf
        depth[:] = 0.0
        valid = d_float > 0.5
        np.divide(self.baseline * self.rect_fx, d_float, out=depth, where=valid)

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
        """Find the closest (highest) object that stands above the ground.

        Strategy: the ground plane is the dominant surface at some depth.
        The RC car / person is closer to the camera (smaller depth) than
        the ground.  We find the minimum valid depth region and track its
        centroid.

        1. Compute ground depth as the mode/median of all valid pixels
        2. Threshold for pixels significantly closer than ground
        3. Find the largest contour in that foreground mask

        Returns (cx, cy, mean_depth) or None.
        """
        # All pixels in the valid depth band
        valid_mask = (depth > self.depth_min) & (depth < self.depth_max)
        if not np.any(valid_mask):
            return None

        # Estimate ground depth as median of all valid pixels
        ground_depth = float(np.median(depth[valid_mask]))

        # Foreground = pixels at least height_threshold closer than ground
        height_thresh = self.get_parameter('height_threshold').value
        fg_max = ground_depth - height_thresh
        if fg_max <= self.depth_min:
            return None

        fg_mask = ((depth > self.depth_min)
                   & (depth < fg_max)).astype(np.uint8) * 255

        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Sort by area descending, pick largest foreground blob
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

            # Median depth within blob
            blob_mask = np.zeros(depth.shape, dtype=np.uint8)
            cv2.drawContours(blob_mask, [contour], -1, 255, -1)
            blob_depths = depth[(blob_mask > 0) & (depth > self.depth_min)]
            if len(blob_depths) == 0:
                continue

            return (cx, cy, float(np.median(blob_depths)))

        return None

    # ------------------------------------------------------------------
    # Debug visualisation
    # ------------------------------------------------------------------
    def _publish_debug(self, depth, detection, header):
        import time as _time
        h, w = depth.shape[:2]

        # -- FPS tracking -------------------------------------------------
        self._frame_count += 1
        now = _time.time()
        if self._fps_timer is None:
            self._fps_timer = now
        elif self._frame_count % 10 == 0:
            self._actual_fps = 10.0 / max(now - self._fps_timer, 1e-6)
            self._fps_timer = now

        # -- Depth colormap -----------------------------------------------
        # Normalize depth to [depth_min, depth_max] so the colormap contrast
        # is concentrated on the detection band (close objects pop vs ground).
        d_vis = np.zeros(depth.shape, dtype=np.uint8)
        valid = depth > 0
        if np.any(valid):
            d_range = max(self.depth_max - self.depth_min, 0.01)
            d_vis = ((depth - self.depth_min) / d_range * 255.0
                     ).clip(0, 255).astype(np.uint8)
            d_vis[~valid] = 0

        d_color = cv2.applyColorMap(d_vis, cv2.COLORMAP_TURBO)
        # Black out invalid pixels so they don't distract
        d_color[~valid] = 0
        WHITE = (255, 255, 255)
        GREY = (180, 180, 180)
        F = cv2.FONT_HERSHEY_SIMPLEX
        S = 0.28  # font scale for 212x200
        T = 1     # thickness

        # -- Image center crosshair (drone nadir) -------------------------
        cv2.drawMarker(d_color, (w // 2, h // 2), WHITE,
                        cv2.MARKER_CROSS, 8, 1)

        # -- Detection marker + control vector ----------------------------
        if detection is not None:
            cx, cy, d = detection
            icx, icy = int(cx), int(cy)
            cv2.circle(d_color, (icx, icy), 5, WHITE, 1)
            cv2.putText(d_color, f'{d:.2f}m', (icx + 7, icy), F, S, WHITE, T)
            cv2.arrowedLine(d_color, (w // 2, h // 2), (icx, icy),
                            WHITE, 1, tipLength=0.15)

        # -- Top-left: status + flight state + FPS ------------------------
        track_color = WHITE if self.tracking_status == 'TRACKING' else (0, 0, 255)
        cv2.putText(d_color, self.tracking_status, (3, 10), F, 0.3, track_color, T)
        cv2.putText(d_color, f'flight: {self._flight_state}', (3, 20), F, S, WHITE, T)
        cv2.putText(d_color, f'fps: {self._actual_fps:.1f}', (3, 30), F, S, WHITE, T)

        # -- Bottom: body offset, pose, setpoint --------------------------
        y_text = h - 36

        if self.tracked_point is not None:
            xb, yb, zb = self.tracked_point
            cv2.putText(d_color,
                        f'fwd={xb:+.2f} rgt={yb:+.2f} dwn={-zb:.2f}m',
                        (3, y_text), F, S, WHITE, T)
        y_text += 10

        if self._current_pose is not None:
            p = self._current_pose.pose.position
            cv2.putText(d_color,
                        f'pos:{p.x:+.1f},{p.y:+.1f},{p.z:+.1f}',
                        (3, y_text), F, S, WHITE, T)
        y_text += 10

        if self._current_setpoint is not None:
            s = self._current_setpoint.pose.position
            cv2.putText(d_color,
                        f'spt:{s.x:+.1f},{s.y:+.1f},{s.z:+.1f}',
                        (3, y_text), F, S, WHITE, T)
        y_text += 10

        # -- Publish ROS debug image --------------------------------------
        debug_msg = self.bridge.cv2_to_imgmsg(d_color, encoding='bgr8')
        debug_msg.header = header
        self.pub_debug.publish(debug_msg)

        # -- Write frame to video via Jetson HW encoder --------------------
        if self.record_video:
            if self._vid_writer is None:
                vid_fps = self._actual_fps if self._actual_fps > 1.0 else 15.0
                gst_pipe = (
                    f'appsrc ! video/x-raw,format=BGR ! videoconvert ! '
                    f'video/x-raw,format=BGRx ! nvvidconv ! '
                    f'video/x-raw(memory:NVMM),format=NV12 ! '
                    f'nvv4l2h264enc bitrate=2000000 preset-level=1 ! '
                    f'h264parse ! matroskamux ! '
                    f'filesink location={self._vid_path}'
                )
                self._vid_writer = cv2.VideoWriter(
                    gst_pipe, cv2.CAP_GSTREAMER,
                    0, vid_fps, (w, h))
                if not self._vid_writer.isOpened():
                    self.get_logger().warn(
                        'GStreamer HW encoder failed, falling back to CPU H264')
                    self._vid_writer = cv2.VideoWriter(
                        self._vid_path, cv2.VideoWriter_fourcc(*'H264'),
                        vid_fps, (w, h))
                self.get_logger().info(
                    f'Video writer opened: {w}x{h} @ {vid_fps:.1f}fps')
            self._vid_writer.write(d_color)

    # ------------------------------------------------------------------
    # Video recording helpers
    # ------------------------------------------------------------------
    def _prune_old_videos(self, keep=2):
        """Keep only the most recent `keep` tracking videos in video_dir."""
        vids = sorted(
            [f for f in os.listdir(self.video_dir)
             if f.startswith('tracking_') and f.endswith('.mkv')])
        for old in vids[:-keep]:
            path = os.path.join(self.video_dir, old)
            try:
                os.remove(path)
                self.get_logger().info(f'Pruned old video: {old}')
            except OSError:
                pass

    def _signal_handler(self, signum, frame):
        """Handle SIGINT/SIGTERM — release video writer before exit."""
        if self._vid_writer is not None:
            self._vid_writer.release()
            self.get_logger().info(f'Video saved: {self._vid_path}')
        raise SystemExit(0)

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
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node._vid_writer is not None:
            node._vid_writer.release()
            node.get_logger().info(f'Video saved: {node._vid_path}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
