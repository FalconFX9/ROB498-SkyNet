#!/usr/bin/env python3
"""
Waypoint Navigation Node for ROB498 Flight Training Exercise #3.

Extends the proven stationkeeping pattern (FT2) with sequential waypoint
navigation.  Supports VICON (Part I) and T265-only (Part II) operation.

Flight state machine:
    IDLE  ->  LAUNCH  ->  TEST  ->  LAND  |  ABORT

During TEST the drone flies to each waypoint in sequence.  A waypoint is
"reached" when the drone enters the 40 cm radius sphere.  After all
waypoints are visited the drone holds position at the last one.

For Part II (T265-only) a Vicon→T265 translation offset is computed at
TEST start using the last known Vicon pose and the current T265 pose.
Both frames are already ENU-oriented, so only a translation is needed.

Course interface (service servers):
    {drone_id}/comm/launch     (Trigger) — Arm, OFFBOARD, ascend
    {drone_id}/comm/test       (Trigger) — Begin waypoint navigation
    {drone_id}/comm/land       (Trigger) — Smooth descent
    {drone_id}/comm/abort      (Trigger) — Emergency disarm

Waypoint subscriber:
    {drone_id}/comm/waypoints  (PoseArray) — 4 waypoints (+ optional
    leading pose = current Vicon position for Part II frame alignment)
"""

import numpy as np
import tf_transformations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import PoseStamped, PoseArray, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from std_srvs.srv import Trigger


class WaypointNode(Node):
    """Waypoint navigation controller for Flight Test #3."""

    def __init__(self):
        # Course skeleton requires node name = drone_id
        super().__init__('rob498_drone_10')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('drone_id',              'rob498_drone_10')
        self.declare_parameter('takeoff_altitude',      0.5)     # m
        self.declare_parameter('setpoint_rate',         20.0)    # Hz (>2 for PX4)
        self.declare_parameter('vision_pose_rate',      30.0)    # Hz
        self.declare_parameter('descent_speed',         0.15)    # m/s
        self.declare_parameter('offboard_wait',         1.5)     # s before OFFBOARD
        self.declare_parameter('mode_request_interval', 2.0)     # s between retries
        self.declare_parameter('vicon_topic',           '')       # empty → Part II
        self.declare_parameter('vicon_timeout',         0.5)     # s
        self.declare_parameter('vicon_yaw_offset',      0.0)     # rad — additional yaw trim on top of body correction
        self.declare_parameter('t265_z_offset',         0.0)     # m (calibration)
        self.declare_parameter('reach_radius',          0.4)     # 40 cm sphere
        self.declare_parameter('log_euler',             False)   # Log RPY at ~2 Hz

        drone_id               = self.get_parameter('drone_id').value
        self.takeoff_altitude  = self.get_parameter('takeoff_altitude').value
        setpoint_rate          = self.get_parameter('setpoint_rate').value
        vision_rate            = self.get_parameter('vision_pose_rate').value
        self.descent_speed     = self.get_parameter('descent_speed').value
        self.offboard_wait     = self.get_parameter('offboard_wait').value
        self.mode_req_interval = self.get_parameter('mode_request_interval').value
        vicon_topic            = self.get_parameter('vicon_topic').value
        self.vicon_timeout     = self.get_parameter('vicon_timeout').value
        self.vicon_yaw_offset  = self.get_parameter('vicon_yaw_offset').value
        self.t265_z_offset     = self.get_parameter('t265_z_offset').value
        self.reach_radius      = self.get_parameter('reach_radius').value
        self.log_euler         = self.get_parameter('log_euler').value
        self._euler_vicon_ctr  = 0
        self._euler_vision_ctr = 0

        # Debug RPY publishers (created only when log_euler is enabled)
        if self.log_euler:
            self._pub_vicon_raw_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vicon_raw_rpy', 10)
            self._pub_vicon_cor_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vicon_cor_rpy', 10)
            self._pub_vision_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vision_rpy', 10)

        # ── Flight state ────────────────────────────────────────────────
        self.flight_state       = 'IDLE'
        self.mavros_state       = None
        self.hover_pose         = None       # PoseStamped held during hover
        self.t265_pose          = None
        self.t265_stamp         = None
        self.vicon_pose         = None
        self.vicon_stamp        = None
        self.launch_time        = None
        self.land_start_time    = None
        self.land_start_alt     = None
        self.last_mode_req_time = None
        self.last_arm_req_time  = None
        self._hover_logged      = False
        self._offboard_achieved = False
        self._raw_t265_z        = None

        # ── Waypoint state ──────────────────────────────────────────────
        self.waypoints              = None    # Nx3 numpy, Vicon frame
        self.waypoints_received     = False
        self.current_wp_index       = 0
        self.wp_target              = None    # PoseStamped in active frame
        self.all_waypoints_reached  = False
        self.vicon_ref_pos          = None    # from PoseArray (optional)

        # Vicon→T265 translation: wp_t265 = wp_vicon − frame_offset
        self.frame_offset          = np.zeros(3)
        self.frame_offset_computed = False
        self.use_vicon             = bool(vicon_topic)

        # ── Vicon body-frame orientation correction ─────────────────────
        # Vicon global frame is already ENU — position needs no rotation.
        # The Vicon rigid body axes don't match the drone's actual body
        # axes.  Measured transform (Vicon body → drone body):
        #   X_drone = −Z_vicon_body
        #   Y_drone = −Y_vicon_body
        #   Z_drone = −X_vicon_body
        # corrected_quat = q_body_correction * vicon_quat
        R_body = np.array([
            [ 0,  0, -1],
            [ 0, -1,  0],
            [-1,  0,  0],
        ])
        self._q_vicon_body_correction = tf_transformations.quaternion_from_matrix(
            np.vstack([
                np.hstack([R_body, np.zeros((3, 1))]),
                [0, 0, 0, 1],
            ])
        )
        # Optional additional yaw trim (from launch param) on top of body correction
        if self.vicon_yaw_offset != 0.0:
            q_yaw_trim = tf_transformations.quaternion_from_euler(
                0.0, 0.0, self.vicon_yaw_offset
            )
            self._q_vicon_body_correction = tf_transformations.quaternion_multiply(
                q_yaw_trim, self._q_vicon_body_correction
            )
            self.get_logger().info(
                f'Vicon body correction + yaw trim {np.degrees(self.vicon_yaw_offset):.1f}°'
            )
        else:
            self.get_logger().info('Vicon body-frame correction active')

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            State, '/mavros/state',
            self._on_mavros_state, qos_profile_system_default,
        )
        self.create_subscription(
            PoseStamped, '/mary/localization/pose',
            self._on_t265_pose, qos_profile_system_default,
        )
        if vicon_topic:
            self.create_subscription(PoseStamped, vicon_topic, self._on_vicon_pose, 10)
            self.get_logger().info(f'VICON topic: {vicon_topic}')
        else:
            self.get_logger().info('No VICON topic — T265 only (Part II)')

        self.create_subscription(
            PoseArray, f'{drone_id}/comm/waypoints',
            self._on_waypoints, 10,
        )

        # ── Publishers ──────────────────────────────────────────────────
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', qos_profile_system_default,
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile_system_default,
        )
        self.state_pub = self.create_publisher(String, '/mary/comm/flight_state', 10)

        # ── MAVROS service clients ──────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')

        # ── Course service servers ──────────────────────────────────────
        self.create_service(Trigger, f'{drone_id}/comm/launch', self._handle_launch)
        self.create_service(Trigger, f'{drone_id}/comm/test',   self._handle_test)
        self.create_service(Trigger, f'{drone_id}/comm/land',   self._handle_land)
        self.create_service(Trigger, f'{drone_id}/comm/abort',  self._handle_abort)

        # ── Timers ──────────────────────────────────────────────────────
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        self.create_timer(1.0 / vision_rate,   self._vision_pose_loop)
        self.create_timer(0.1,                 self._state_machine_loop)  # 10 Hz

        self.get_logger().info(
            f'WaypointNode ready  drone_id={drone_id}  '
            f'altitude={self.takeoff_altitude}m  '
            f'reach={self.reach_radius}m  '
            f'setpoint={setpoint_rate}Hz  vision={vision_rate}Hz'
        )

    # ━━ Subscriber callbacks ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _on_mavros_state(self, msg):
        self.mavros_state = msg

    def _on_t265_pose(self, msg):
        self._raw_t265_z = msg.pose.position.z
        msg.pose.position.z += self.t265_z_offset
        self.t265_pose  = msg
        self.t265_stamp = self.get_clock().now()

    def _on_vicon_pose(self, msg):
        # Position: Vicon global frame is already ENU — pass through as-is
        # Orientation: apply body-frame correction to align Vicon rigid body
        # axes with the drone's actual body axes
        quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ])
        corrected = tf_transformations.quaternion_multiply(
            self._q_vicon_body_correction, quat
        )

        transformed = PoseStamped()
        transformed.header              = msg.header
        transformed.pose.position       = msg.pose.position  # unchanged
        transformed.pose.orientation.x  = float(corrected[0])
        transformed.pose.orientation.y  = float(corrected[1])
        transformed.pose.orientation.z  = float(corrected[2])
        transformed.pose.orientation.w  = float(corrected[3])

        self.vicon_pose  = transformed
        self.vicon_stamp = self.get_clock().now()

        if self.log_euler:
            stamp = self.get_clock().now().to_msg()
            raw_rpy = np.degrees(tf_transformations.euler_from_quaternion(
                [quat[0], quat[1], quat[2], quat[3]]))
            cor_rpy = np.degrees(tf_transformations.euler_from_quaternion(
                [corrected[0], corrected[1], corrected[2], corrected[3]]))

            msg_raw = Vector3Stamped()
            msg_raw.header.stamp = stamp
            msg_raw.vector.x, msg_raw.vector.y, msg_raw.vector.z = \
                float(raw_rpy[0]), float(raw_rpy[1]), float(raw_rpy[2])
            self._pub_vicon_raw_rpy.publish(msg_raw)

            msg_cor = Vector3Stamped()
            msg_cor.header.stamp = stamp
            msg_cor.vector.x, msg_cor.vector.y, msg_cor.vector.z = \
                float(cor_rpy[0]), float(cor_rpy[1]), float(cor_rpy[2])
            self._pub_vicon_cor_rpy.publish(msg_cor)

    def _on_waypoints(self, msg):
        """Store waypoints from ground control (first call only).

        If the PoseArray has more than 4 poses, the first is treated as
        the drone's current Vicon position (for Part II frame alignment)
        and the rest are waypoints.
        """
        if self.waypoints_received:
            return
        self.waypoints_received = True

        poses = msg.poses
        if len(poses) > 4:
            # First pose = current Vicon position for frame alignment
            self.vicon_ref_pos = np.array([
                poses[0].position.x, poses[0].position.y, poses[0].position.z,
            ])
            waypoint_poses = poses[1:]
            self.get_logger().info(f'Vicon ref position: {self.vicon_ref_pos}')
        else:
            waypoint_poses = poses

        self.waypoints = np.array([
            [p.position.x, p.position.y, p.position.z] for p in waypoint_poses
        ])
        self.get_logger().info(
            f'Received {len(self.waypoints)} waypoints:\n{self.waypoints}'
        )

    # ━━ Pose source management ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _get_active_pose(self):
        """Return best available pose: fresh VICON > T265 > None."""
        now = self.get_clock().now()
        if self.vicon_pose is not None and self.vicon_stamp is not None:
            age = (now - self.vicon_stamp).nanoseconds * 1e-9
            if age < self.vicon_timeout:
                return self.vicon_pose
        return self.t265_pose

    def _get_current_position(self):
        """Current position as numpy [x, y, z], or None."""
        pose = self._get_active_pose()
        if pose is None:
            return None
        p = pose.pose.position
        return np.array([p.x, p.y, p.z])

    # ━━ Waypoint helpers ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _compute_frame_offset(self):
        """Compute Vicon→T265 translation for Part II.

        Part I (Vicon available throughout): offset = 0.
        Part II: offset = vicon_pos − t265_pos at TEST time.
        """
        if self.use_vicon:
            self.frame_offset = np.zeros(3)
            self.frame_offset_computed = True
            return True

        if self.t265_pose is None:
            self.get_logger().warn('Frame offset: no T265 pose available')
            return False

        t265_pos = np.array([
            self.t265_pose.pose.position.x,
            self.t265_pose.pose.position.y,
            self.t265_pose.pose.position.z,
        ])

        # Prefer Vicon ref from PoseArray, fall back to last subscription
        if self.vicon_ref_pos is not None:
            vicon_pos = self.vicon_ref_pos
        elif self.vicon_pose is not None:
            vicon_pos = np.array([
                self.vicon_pose.pose.position.x,
                self.vicon_pose.pose.position.y,
                self.vicon_pose.pose.position.z,
            ])
        else:
            self.get_logger().warn('Frame offset: no Vicon reference')
            return False

        self.frame_offset = vicon_pos - t265_pos
        self.frame_offset_computed = True
        self.get_logger().info(
            f'Frame offset: {self.frame_offset}  '
            f'(Vicon={vicon_pos}, T265={t265_pos})'
        )
        return True

    def _wp_in_active_frame(self, wp_vicon):
        """Transform a Vicon-frame waypoint into the active nav frame."""
        if self.use_vicon:
            return wp_vicon
        return wp_vicon - self.frame_offset

    def _update_wp_target(self):
        """Build a PoseStamped for the current waypoint."""
        if self.waypoints is None or self.current_wp_index >= len(self.waypoints):
            return
        wp = self._wp_in_active_frame(self.waypoints[self.current_wp_index])

        self.wp_target = PoseStamped()
        self.wp_target.pose.position.x = float(wp[0])
        self.wp_target.pose.position.y = float(wp[1])
        self.wp_target.pose.position.z = float(wp[2])
        # Preserve heading from hover pose
        if self.hover_pose is not None:
            self.wp_target.pose.orientation = self.hover_pose.pose.orientation
        else:
            self.wp_target.pose.orientation.w = 1.0

    def _check_wp_reached(self):
        """True if drone is inside the reach sphere of current waypoint."""
        if self.waypoints is None or self.current_wp_index >= len(self.waypoints):
            return False
        pos = self._get_current_position()
        if pos is None:
            return False
        wp = self._wp_in_active_frame(self.waypoints[self.current_wp_index])
        return float(np.linalg.norm(pos - wp)) < self.reach_radius

    # ━━ Vision pose relay ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _vision_pose_loop(self):
        """Relay active pose to MAVROS for PX4 EKF2 fusion."""
        pose = self._get_active_pose()
        if pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose            = pose.pose
        self.vision_pose_pub.publish(msg)

        if self.log_euler:
            q = pose.pose.orientation
            rpy = np.degrees(tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]))
            msg_v = Vector3Stamped()
            msg_v.header.stamp = msg.header.stamp
            msg_v.vector.x, msg_v.vector.y, msg_v.vector.z = \
                float(rpy[0]), float(rpy[1]), float(rpy[2])
            self._pub_vision_rpy.publish(msg_v)

    # ━━ Setpoint publishing ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _setpoint_loop(self):
        """Publish position setpoints at a fixed rate.

        TEST with active wp_target → fly to current waypoint.
        IDLE / LAUNCH / TEST (no target) → hold hover_pose.
        LAND → ramp Z toward ground, hold X/Y.
        ABORT → stop publishing.
        """
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.flight_state == 'TEST' and self.wp_target is not None:
            # ── Navigate to current waypoint ────────────────────────
            msg.pose = self.wp_target.pose

        elif self.flight_state in ('IDLE', 'LAUNCH', 'TEST'):
            # ── Hold hover position ─────────────────────────────────
            if self.hover_pose is not None:
                msg.pose.position.x    = self.hover_pose.pose.position.x
                msg.pose.position.y    = self.hover_pose.pose.position.y
                msg.pose.position.z    = self.hover_pose.pose.position.z
                msg.pose.orientation.x = self.hover_pose.pose.orientation.x
                msg.pose.orientation.y = self.hover_pose.pose.orientation.y
                msg.pose.orientation.z = self.hover_pose.pose.orientation.z
                msg.pose.orientation.w = self.hover_pose.pose.orientation.w
            else:
                pose = self._get_active_pose()
                if pose is not None:
                    msg.pose = pose.pose
                else:
                    msg.pose.position.z    = self.takeoff_altitude
                    msg.pose.orientation.w = 1.0

        elif self.flight_state == 'LAND':
            # ── Descend while holding X/Y/heading ───────────────────
            if self.hover_pose is not None:
                msg.pose.position.x    = self.hover_pose.pose.position.x
                msg.pose.position.y    = self.hover_pose.pose.position.y
                msg.pose.orientation.x = self.hover_pose.pose.orientation.x
                msg.pose.orientation.y = self.hover_pose.pose.orientation.y
                msg.pose.orientation.z = self.hover_pose.pose.orientation.z
                msg.pose.orientation.w = self.hover_pose.pose.orientation.w
            if self.land_start_time is not None:
                elapsed  = (self.get_clock().now() - self.land_start_time).nanoseconds * 1e-9
                target_z = self.land_start_alt - self.descent_speed * elapsed
                msg.pose.position.z = max(target_z, 0.0)
            else:
                msg.pose.position.z = 0.0

        elif self.flight_state == 'ABORT':
            self._broadcast_state()
            return

        self.setpoint_pub.publish(msg)
        self._broadcast_state()

    def _broadcast_state(self):
        msg      = String()
        msg.data = self.flight_state
        self.state_pub.publish(msg)

    # ━━ State machine (10 Hz) ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _state_machine_loop(self):
        # ── RC override / external disarm ───────────────────────────
        if (self._offboard_achieved
                and self.flight_state in ('LAUNCH', 'TEST', 'LAND')
                and self.mavros_state is not None):
            if self.mavros_state.mode != 'OFFBOARD':
                self.get_logger().info(
                    f'RC override (mode={self.mavros_state.mode}) — releasing'
                )
                self._reset_to_idle()
                return
            if not self.mavros_state.armed:
                self.get_logger().info('External disarm — resetting to IDLE')
                self._reset_to_idle()
                return

        if self.flight_state == 'LAUNCH':
            self._tick_launch()
        elif self.flight_state == 'TEST':
            self._tick_test()
        elif self.flight_state == 'LAND':
            self._tick_land()

    def _tick_launch(self):
        """Request OFFBOARD + arm after setpoints have streamed."""
        if self.mavros_state is None:
            return
        now     = self.get_clock().now()
        elapsed = (now - self.launch_time).nanoseconds * 1e-9

        if elapsed < self.offboard_wait:
            return

        if self.mavros_state.mode != 'OFFBOARD':
            if (self.last_mode_req_time is None
                    or (now - self.last_mode_req_time).nanoseconds * 1e-9
                    > self.mode_req_interval):
                self.get_logger().info('Requesting OFFBOARD mode...')
                self._request_mode('OFFBOARD')
                self.last_mode_req_time = now
            return

        if not self.mavros_state.armed:
            if (self.last_arm_req_time is None
                    or (now - self.last_arm_req_time).nanoseconds * 1e-9
                    > self.mode_req_interval):
                self.get_logger().info('Requesting arming...')
                self._request_arm(True)
                self.last_arm_req_time = now
            return

        self._offboard_achieved = True
        if not self._hover_logged:
            self.get_logger().info('Armed in OFFBOARD — hovering, ready for TEST')
            self._hover_logged = True

    def _tick_test(self):
        """Check waypoint reach at 10 Hz, advance when inside sphere."""
        if self.all_waypoints_reached or self.waypoints is None:
            return

        # Log distance to current waypoint (throttled to ~2 Hz)
        if not hasattr(self, '_dist_log_counter'):
            self._dist_log_counter = 0
        self._dist_log_counter += 1
        pos = self._get_current_position()
        if pos is not None and self._dist_log_counter % 5 == 0:
            wp = self._wp_in_active_frame(self.waypoints[self.current_wp_index])
            dist = float(np.linalg.norm(pos - wp))
            self.get_logger().info(
                f'WP {self.current_wp_index + 1}/{len(self.waypoints)}  '
                f'dist={dist:.2f}m  pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})'
            )

        if not self._check_wp_reached():
            return

        # ── Waypoint reached ────────────────────────────────────────
        self.get_logger().info(
            f'>>> Waypoint {self.current_wp_index + 1}/{len(self.waypoints)} REACHED!'
        )
        self.current_wp_index += 1

        if self.current_wp_index >= len(self.waypoints):
            self.all_waypoints_reached = True
            self.get_logger().info('All waypoints reached — holding position')
            # Hold at last waypoint
            if self.wp_target is not None:
                self.hover_pose = PoseStamped()
                self.hover_pose.pose = self.wp_target.pose
            self.wp_target = None
        else:
            self._update_wp_target()
            wp = self.waypoints[self.current_wp_index]
            self.get_logger().info(
                f'Next: waypoint {self.current_wp_index + 1}  '
                f'({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})'
            )

    def _tick_land(self):
        """Monitor altitude during landing."""
        pose = self._get_active_pose()
        if pose is not None:
            if pose.pose.position.z <= 0.02 and not getattr(self, '_landed_logged', False):
                self.get_logger().info('On the ground — disarm manually or call abort')
                self._landed_logged = True

    def _reset_to_idle(self):
        self.flight_state          = 'IDLE'
        self.hover_pose            = None
        self.land_start_time       = None
        self.land_start_alt        = None
        self._hover_logged         = False
        self._offboard_achieved    = False
        self._landed_logged        = False
        self.current_wp_index      = 0
        self.wp_target             = None
        self.all_waypoints_reached = False
        self.frame_offset_computed = False

    # ━━ Course service handlers ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _handle_launch(self, request, response):
        if self.flight_state not in ('IDLE', 'ABORT'):
            response.success = False
            response.message = f'Cannot launch: state={self.flight_state}'
            return response

        # Capture hover reference (current X/Y + takeoff Z + heading)
        pose = self._get_active_pose()
        if pose is not None:
            self.hover_pose = PoseStamped()
            self.hover_pose.pose.position.x   = pose.pose.position.x
            self.hover_pose.pose.position.y   = pose.pose.position.y
            self.hover_pose.pose.position.z   = self.takeoff_altitude
            self.hover_pose.pose.orientation  = pose.pose.orientation
            self.get_logger().info(
                f'Hover target: ({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f}, {self.takeoff_altitude:.2f})'
            )
        else:
            self.get_logger().warn('No pose at launch — using default setpoint')

        self.flight_state       = 'LAUNCH'
        self.launch_time        = self.get_clock().now()
        self.last_mode_req_time = None
        self.last_arm_req_time  = None
        self._hover_logged      = False
        self._offboard_achieved = False

        wp_status = f'{len(self.waypoints)} waypoints loaded' if self.waypoints is not None else 'no waypoints yet'
        self.get_logger().info(f'LAUNCH — ascending to {self.takeoff_altitude}m  ({wp_status})')
        response.success = True
        response.message = f'Launching to {self.takeoff_altitude}m'
        return response

    def _handle_test(self, request, response):
        if self.flight_state not in ('LAUNCH', 'TEST'):
            response.success = False
            response.message = f'Cannot test: state={self.flight_state}'
            return response

        if not self.waypoints_received or self.waypoints is None:
            self.get_logger().warn('TEST but no waypoints — will hover')
            self.flight_state = 'TEST'
            response.success  = True
            response.message  = 'Test active (no waypoints — hovering)'
            return response

        # Compute Vicon→T265 frame offset for Part II
        if not self._compute_frame_offset():
            self.get_logger().warn('Frame offset failed — using zero offset')

        # Start waypoint navigation from the first waypoint
        self.current_wp_index      = 0
        self.all_waypoints_reached = False
        self._dist_log_counter     = 0
        self._update_wp_target()

        self.flight_state = 'TEST'
        wp = self.waypoints[0]
        self.get_logger().info(
            f'TEST — navigating {len(self.waypoints)} waypoints  '
            f'first: ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})'
        )
        response.success = True
        response.message = f'Waypoint navigation active ({len(self.waypoints)} wp)'
        return response

    def _handle_land(self, request, response):
        if self.flight_state in ('IDLE', 'ABORT'):
            response.success = False
            response.message = f'Cannot land: state={self.flight_state}'
            return response

        pose = self._get_active_pose()
        self.land_start_alt  = pose.pose.position.z if pose else self.takeoff_altitude
        self.land_start_time = self.get_clock().now()
        self.flight_state    = 'LAND'
        self.wp_target       = None   # stop waypoint navigation

        # Hold current X/Y during descent
        if pose is not None:
            if self.hover_pose is None:
                self.hover_pose = PoseStamped()
                self.hover_pose.pose.orientation = pose.pose.orientation
            self.hover_pose.pose.position.x = pose.pose.position.x
            self.hover_pose.pose.position.y = pose.pose.position.y

        self.get_logger().info(
            f'LAND — descending from {self.land_start_alt:.2f}m '
            f'at {self.descent_speed} m/s'
        )
        response.success = True
        response.message = 'Landing initiated'
        return response

    def _handle_abort(self, request, response):
        self.get_logger().warn('ABORT — emergency disarm')
        self.flight_state       = 'ABORT'
        self.hover_pose         = None
        self.land_start_time    = None
        self.wp_target          = None
        self._offboard_achieved = False
        self._request_arm(False)

        response.success = True
        response.message = 'Emergency stop executed'
        return response

    # ━━ MAVROS helpers ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _request_mode(self, mode):
        if not self.set_mode_client.service_is_ready():
            self.get_logger().warn('set_mode service not ready')
            return
        req             = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def _request_arm(self, arm):
        if not self.arming_client.service_is_ready():
            self.get_logger().warn('arming service not ready')
            return
        req       = CommandBool.Request()
        req.value = arm
        self.arming_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
