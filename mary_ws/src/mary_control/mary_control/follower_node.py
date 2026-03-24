#!/usr/bin/env python3
"""
Person Follower Node for MARY Drone

Follows a tracked target using position setpoints.  Built on the proven
waypoint_node.py patterns (FT2/FT3) with tracking-specific logic.

Flight state machine:
    IDLE  ->  LAUNCH  ->  FOLLOW  <->  HOVER  ->  LAND  |  ABORT

Pose source handoff:
    In part=1 (VICON mode), the drone takes off using VICON for reliable
    position estimation.  Once airborne and stable, it can hand off to T265
    as the primary pose source (with a Z offset calibrated from VICON).
    This can happen automatically after Z offset calibration + stable hover
    (auto_handoff=true) or via a manual service call.

    After handoff, VICON is kept as an emergency fallback only — if T265
    drops out, VICON will be used temporarily until T265 recovers.

The stereo_tracker_node publishes the target's 3D offset in body frame.
This node converts that to world-frame position setpoints for MAVROS.

Course interface (service servers):
    {drone_id}/comm/launch       (Trigger) -- Arm, OFFBOARD, ascend
    {drone_id}/comm/test         (Trigger) -- Begin following (kept as "test"
                                              for course interface compat)
    {drone_id}/comm/land         (Trigger) -- Smooth descent
    {drone_id}/comm/abort        (Trigger) -- Emergency disarm
    {drone_id}/comm/switch_pose  (Trigger) -- Hand off pose source to T265
"""

import numpy as np
import tf_transformations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import PoseStamped, PointStamped, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from std_srvs.srv import Trigger


class FollowerNode(Node):
    """Position-setpoint follower controller for MARY person tracking."""

    def __init__(self):
        # Course skeleton requires node name = drone_id
        super().__init__('rob498_drone_10')

        # -- Parameters -------------------------------------------------------
        self.declare_parameter('drone_id',              'rob498_drone_10')
        self.declare_parameter('takeoff_altitude',      1.5)      # m
        self.declare_parameter('target_altitude_above', 1.0)      # m above person
        self.declare_parameter('setpoint_rate',         20.0)     # Hz
        self.declare_parameter('vision_pose_rate',      30.0)     # Hz
        self.declare_parameter('descent_speed',         0.15)     # m/s
        self.declare_parameter('offboard_wait',         1.5)      # s
        self.declare_parameter('mode_request_interval', 2.0)      # s
        self.declare_parameter('vicon_topic',           '')
        self.declare_parameter('vicon_timeout',         0.5)      # s
        self.declare_parameter('vicon_yaw_offset',      0.0)      # rad
        self.declare_parameter('t265_z_offset',         0.0)      # m
        self.declare_parameter('hover_timeout',         3.0)      # s -> auto-land
        self.declare_parameter('max_follow_speed',      1.5)      # m/s rate limit
        self.declare_parameter('auto_handoff',          False)    # auto switch to T265 after Z cal
        self.declare_parameter('handoff_stable_secs',   3.0)     # s of stable hover before auto handoff
        self.declare_parameter('log_euler',             False)
        self.declare_parameter('debug_follow',          False)    # skip arming, go straight to FOLLOW

        drone_id               = self.get_parameter('drone_id').value
        self.takeoff_altitude  = self.get_parameter('takeoff_altitude').value
        self.target_alt_above  = self.get_parameter('target_altitude_above').value
        self.setpoint_rate     = self.get_parameter('setpoint_rate').value
        setpoint_rate          = self.setpoint_rate
        vision_rate            = self.get_parameter('vision_pose_rate').value
        self.descent_speed     = self.get_parameter('descent_speed').value
        self.offboard_wait     = self.get_parameter('offboard_wait').value
        self.mode_req_interval = self.get_parameter('mode_request_interval').value
        vicon_topic            = self.get_parameter('vicon_topic').value
        self.vicon_timeout     = self.get_parameter('vicon_timeout').value
        self.vicon_yaw_offset  = self.get_parameter('vicon_yaw_offset').value
        self.t265_z_offset     = self.get_parameter('t265_z_offset').value
        self.hover_timeout     = self.get_parameter('hover_timeout').value
        self.max_follow_speed  = self.get_parameter('max_follow_speed').value
        self.auto_handoff      = self.get_parameter('auto_handoff').value
        self.handoff_stable_s  = self.get_parameter('handoff_stable_secs').value
        self.log_euler         = self.get_parameter('log_euler').value
        self.debug_follow      = self.get_parameter('debug_follow').value

        # Debug RPY publishers
        if self.log_euler:
            self._pub_vicon_raw_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vicon_raw_rpy', 10)
            self._pub_vicon_cor_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vicon_cor_rpy', 10)
            self._pub_vision_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vision_rpy', 10)

        # -- Flight state -----------------------------------------------------
        self.flight_state       = 'IDLE'
        self.mavros_state       = None
        self.hover_pose         = None       # PoseStamped
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

        # -- T265 frame alignment ------------------------------------------------
        # T265 is odometry (starts at origin). When it comes online mid-flight,
        # we compute a Z offset = VICON_z - T265_z so the T265 altitude aligns
        # with the VICON world frame. X/Y are left as-is (T265 odometry origin).
        self.t265_z_auto_offset      = None   # float, computed once
        self.t265_z_auto_offset_done = False

        # -- Tracking state ---------------------------------------------------
        self.tracking_offset    = None       # latest PointStamped from tracker
        self.tracking_status    = 'LOST'
        self.hover_start_time   = None       # when HOVER was entered
        self.follow_target      = None       # PoseStamped in world frame
        self.last_good_target   = None       # last valid target for HOVER hold

        # -- Vicon body-frame correction (same as waypoint_node) ---------------
        self.use_vicon = bool(vicon_topic)
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
        if self.vicon_yaw_offset != 0.0:
            q_yaw = tf_transformations.quaternion_from_euler(
                0.0, 0.0, self.vicon_yaw_offset)
            self._q_vicon_body_correction = \
                tf_transformations.quaternion_multiply(
                    q_yaw, self._q_vicon_body_correction)
            self.get_logger().info(
                f'Vicon body correction + yaw trim '
                f'{np.degrees(self.vicon_yaw_offset):.1f} deg')
        else:
            self.get_logger().info('Vicon body-frame correction active')

        # -- Pose source handoff -------------------------------------------------
        # Starts on VICON (part=1) or T265 (part=2). Can switch mid-flight.
        #   'vicon'  -> VICON primary, T265 fallback
        #   't265'   -> T265 primary, VICON emergency fallback only
        self._pose_source = 'vicon' if self.use_vicon else 't265'
        self._handoff_done = not self.use_vicon   # no handoff needed in T265-only
        self._stable_hover_start = None           # for auto-handoff timing

        # -- Subscribers ------------------------------------------------------
        self.create_subscription(
            State, '/mavros/state',
            self._on_mavros_state, qos_profile_system_default)
        self.create_subscription(
            PoseStamped, '/mary/localization/pose',
            self._on_t265_pose, qos_profile_system_default)

        if vicon_topic:
            self.create_subscription(
                PoseStamped, vicon_topic, self._on_vicon_pose, 10)
            self.get_logger().info(f'VICON topic: {vicon_topic}')
        else:
            self.get_logger().info('No VICON -- T265 only')

        # Tracking input
        self.create_subscription(
            PointStamped, '/mary/tracking/target',
            self._on_tracking_target, 10)
        self.create_subscription(
            String, '/mary/tracking/status',
            self._on_tracking_status, 10)

        # -- Publishers -------------------------------------------------------
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose',
            qos_profile_system_default)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local',
            qos_profile_system_default)
        self.state_pub = self.create_publisher(
            String, '/mary/comm/flight_state', 10)

        # -- MAVROS service clients -------------------------------------------
        self.arming_client   = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # -- Course service servers -------------------------------------------
        self.create_service(
            Trigger, f'{drone_id}/comm/launch',      self._handle_launch)
        self.create_service(
            Trigger, f'{drone_id}/comm/test',         self._handle_test)
        self.create_service(
            Trigger, f'{drone_id}/comm/land',         self._handle_land)
        self.create_service(
            Trigger, f'{drone_id}/comm/abort',        self._handle_abort)
        self.create_service(
            Trigger, f'{drone_id}/comm/switch_pose',  self._handle_switch_pose)

        # -- Timers -----------------------------------------------------------
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        self.create_timer(1.0 / vision_rate,   self._vision_pose_loop)
        self.create_timer(0.1,                 self._state_machine_loop)

        self.get_logger().info(
            f'FollowerNode ready  drone_id={drone_id}  '
            f'altitude={self.takeoff_altitude}m  '
            f'target_above={self.target_alt_above}m  '
            f'setpoint={setpoint_rate}Hz  vision={vision_rate}Hz  '
            f'pose_source={self._pose_source}  '
            f'auto_handoff={self.auto_handoff}')

        if self.debug_follow:
            self.flight_state = 'FOLLOW'
            self._offboard_achieved = True
            self.hover_pose = PoseStamped()
            self.hover_pose.pose.position.z = self.takeoff_altitude
            self.hover_pose.pose.orientation.w = 1.0
            self.get_logger().warn(
                'DEBUG MODE: skipping arming, starting in FOLLOW state')

    # == Subscriber callbacks ================================================

    def _on_mavros_state(self, msg):
        self.mavros_state = msg

    def _on_t265_pose(self, msg):
        self._raw_t265_z = msg.pose.position.z

        # Auto-compute Z offset on first T265 pose if VICON is available.
        # This aligns T265 altitude to the VICON world frame when T265
        # is started mid-flight.
        if not self.t265_z_auto_offset_done and self.vicon_pose is not None:
            now = self.get_clock().now()
            vicon_age = (now - self.vicon_stamp).nanoseconds * 1e-9
            if vicon_age < self.vicon_timeout:
                vicon_z = self.vicon_pose.pose.position.z
                t265_z = msg.pose.position.z
                self.t265_z_auto_offset = vicon_z - t265_z
                self.t265_z_auto_offset_done = True
                self.get_logger().info(
                    f'T265 Z auto-offset computed: {self.t265_z_auto_offset:.3f}m '
                    f'(VICON={vicon_z:.3f}, T265={t265_z:.3f})')

        # Apply both the manual calibration offset and the auto offset
        msg.pose.position.z += self.t265_z_offset
        if self.t265_z_auto_offset is not None:
            msg.pose.position.z += self.t265_z_auto_offset

        self.t265_pose  = msg
        self.t265_stamp = self.get_clock().now()

    def _on_vicon_pose(self, msg):
        quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ])
        corrected = tf_transformations.quaternion_multiply(
            self._q_vicon_body_correction, quat)

        transformed = PoseStamped()
        transformed.header             = msg.header
        transformed.pose.position      = msg.pose.position
        transformed.pose.orientation.x = float(corrected[0])
        transformed.pose.orientation.y = float(corrected[1])
        transformed.pose.orientation.z = float(corrected[2])
        transformed.pose.orientation.w = float(corrected[3])

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
            msg_raw.vector.x = float(raw_rpy[0])
            msg_raw.vector.y = float(raw_rpy[1])
            msg_raw.vector.z = float(raw_rpy[2])
            self._pub_vicon_raw_rpy.publish(msg_raw)

            msg_cor = Vector3Stamped()
            msg_cor.header.stamp = stamp
            msg_cor.vector.x = float(cor_rpy[0])
            msg_cor.vector.y = float(cor_rpy[1])
            msg_cor.vector.z = float(cor_rpy[2])
            self._pub_vicon_cor_rpy.publish(msg_cor)

    def _on_tracking_target(self, msg):
        self.tracking_offset = msg

    def _on_tracking_status(self, msg):
        self.tracking_status = msg.data

    # == Pose helpers ========================================================

    def _get_active_pose(self):
        """Return best available pose based on current pose source.

        Before handoff (_pose_source == 'vicon'):
            VICON primary, T265 fallback.
        After handoff (_pose_source == 't265'):
            T265 primary, VICON emergency fallback only (if T265 is None).
        """
        now = self.get_clock().now()
        vicon_fresh = (
            self.vicon_pose is not None
            and self.vicon_stamp is not None
            and (now - self.vicon_stamp).nanoseconds * 1e-9 < self.vicon_timeout
        )

        if self._pose_source == 'vicon':
            # Pre-handoff: prefer VICON
            if vicon_fresh:
                return self.vicon_pose
            return self.t265_pose
        else:
            # Post-handoff: prefer T265, VICON only as emergency fallback
            if self.t265_pose is not None:
                return self.t265_pose
            if vicon_fresh:
                self.get_logger().warn(
                    'T265 unavailable after handoff -- using VICON fallback')
                return self.vicon_pose
            return None

    def _get_current_position(self):
        pose = self._get_active_pose()
        if pose is None:
            return None
        p = pose.pose.position
        return np.array([p.x, p.y, p.z])

    def _get_current_yaw(self):
        pose = self._get_active_pose()
        if pose is None:
            return 0.0
        q = pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        return yaw

    # == Tracking -> world frame =============================================

    def _compute_follow_target(self):
        """Convert body-frame tracking offset to world-frame setpoint.

        The stereo_tracker_node publishes (x_body, y_body, z_body) where:
          x_body = forward offset to person
          y_body = rightward offset to person
          z_body = downward offset to person (negative value)

        We rotate the horizontal component by drone yaw to get world-frame
        offset, then compute a setpoint directly above the person.
        """
        if self.tracking_offset is None:
            return None

        pos = self._get_current_position()
        if pos is None:
            return None

        dx_body = self.tracking_offset.point.x  # forward
        dy_body = self.tracking_offset.point.y  # right
        dz_body = self.tracking_offset.point.z  # down (negative)

        # Body frame ≈ world frame (drone flies with fixed heading, no yaw rotation)
        # Negate: offset points drone→person, but we want drone to move toward person
        person_x = pos[0] - dx_body
        person_y = pos[1] - dy_body
        person_z = pos[2] + dz_body   # dz_body is negative

        # Setpoint: directly above person at target altitude
        target = PoseStamped()
        target.pose.position.x = person_x
        target.pose.position.y = person_y
        target.pose.position.z = person_z + self.target_alt_above

        # Preserve current heading
        pose = self._get_active_pose()
        if pose is not None:
            target.pose.orientation = pose.pose.orientation
        else:
            target.pose.orientation.w = 1.0

        return target

    # == Vision pose relay ===================================================

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
            msg_v.vector.x = float(rpy[0])
            msg_v.vector.y = float(rpy[1])
            msg_v.vector.z = float(rpy[2])
            self._pub_vision_rpy.publish(msg_v)

    # == Setpoint publishing =================================================

    def _setpoint_loop(self):
        """Publish position setpoints at fixed rate.

        FOLLOW  -> fly toward tracked person (rate-limited)
        HOVER   -> hold last good target position
        IDLE/LAUNCH -> hold hover_pose
        LAND    -> ramp Z toward ground
        ABORT   -> stop publishing
        """
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.flight_state == 'FOLLOW':
            target = self._compute_follow_target()
            if target is not None:
                # Rate-limit horizontal position change for safety
                pos = self._get_current_position()
                if pos is not None:
                    dx = target.pose.position.x - pos[0]
                    dy = target.pose.position.y - pos[1]
                    dist = np.sqrt(dx*dx + dy*dy)
                    dt = 1.0 / self.setpoint_rate
                    max_step = self.max_follow_speed * dt
                    if dist > max_step:
                        scale = max_step / dist
                        target.pose.position.x = pos[0] + dx * scale
                        target.pose.position.y = pos[1] + dy * scale

                self.follow_target = target
                self.last_good_target = target
                msg.pose = target.pose
            elif self.last_good_target is not None:
                msg.pose = self.last_good_target.pose
            elif self.hover_pose is not None:
                msg.pose = self.hover_pose.pose
            else:
                self._fill_default_setpoint(msg)

        elif self.flight_state == 'HOVER':
            if self.last_good_target is not None:
                msg.pose = self.last_good_target.pose
            elif self.hover_pose is not None:
                msg.pose = self.hover_pose.pose
            else:
                self._fill_default_setpoint(msg)

        elif self.flight_state in ('IDLE', 'LAUNCH'):
            if self.hover_pose is not None:
                msg.pose = self.hover_pose.pose
            else:
                self._fill_default_setpoint(msg)

        elif self.flight_state == 'LAND':
            if self.hover_pose is not None:
                msg.pose.position.x    = self.hover_pose.pose.position.x
                msg.pose.position.y    = self.hover_pose.pose.position.y
                msg.pose.orientation.x = self.hover_pose.pose.orientation.x
                msg.pose.orientation.y = self.hover_pose.pose.orientation.y
                msg.pose.orientation.z = self.hover_pose.pose.orientation.z
                msg.pose.orientation.w = self.hover_pose.pose.orientation.w
            if self.land_start_time is not None:
                elapsed  = (self.get_clock().now()
                            - self.land_start_time).nanoseconds * 1e-9
                target_z = self.land_start_alt - self.descent_speed * elapsed
                msg.pose.position.z = max(target_z, 0.0)
            else:
                msg.pose.position.z = 0.0

        elif self.flight_state == 'ABORT':
            self._broadcast_state()
            return

        self.setpoint_pub.publish(msg)
        self._broadcast_state()

    def _fill_default_setpoint(self, msg):
        """Fall back to current pose or default altitude."""
        pose = self._get_active_pose()
        if pose is not None:
            msg.pose = pose.pose
        else:
            msg.pose.position.z    = self.takeoff_altitude
            msg.pose.orientation.w = 1.0

    def _broadcast_state(self):
        msg      = String()
        msg.data = self.flight_state
        self.state_pub.publish(msg)

    # == State machine (10 Hz) ===============================================

    def _state_machine_loop(self):
        # RC override / external disarm detection
        if (self._offboard_achieved
                and self.flight_state in ('LAUNCH', 'FOLLOW', 'HOVER', 'LAND')
                and self.mavros_state is not None):
            if self.mavros_state.mode != 'OFFBOARD':
                self.get_logger().info(
                    f'RC override (mode={self.mavros_state.mode}) -- releasing')
                self._reset_to_idle()
                return
            if not self.mavros_state.armed:
                self.get_logger().info('External disarm -- resetting to IDLE')
                self._reset_to_idle()
                return

        if self.flight_state == 'LAUNCH':
            self._tick_launch()
        elif self.flight_state == 'FOLLOW':
            self._tick_follow()
        elif self.flight_state == 'HOVER':
            self._tick_hover()
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
            self.get_logger().info(
                'Armed in OFFBOARD -- hovering, ready for TEST/FOLLOW')
            self._hover_logged = True

        # -- Auto handoff from VICON to T265 once stable ----------------------
        if (self.auto_handoff
                and not self._handoff_done
                and self.t265_z_auto_offset_done
                and self.t265_pose is not None):
            if self._stable_hover_start is None:
                self._stable_hover_start = now
            stable_secs = (now - self._stable_hover_start).nanoseconds * 1e-9
            if stable_secs >= self.handoff_stable_s:
                self._do_handoff(
                    f'auto after {stable_secs:.1f}s stable hover')

    def _tick_follow(self):
        """Monitor tracking status during FOLLOW."""
        if self.tracking_status == 'LOST' and self.last_good_target is not None:
            # Only transition to HOVER if we had tracking before and lost it.
            # Don't transition if we never had tracking (e.g. startup).
            self.get_logger().info('Tracking lost -- entering HOVER')
            self.flight_state = 'HOVER'
            self.hover_start_time = self.get_clock().now()
            self.hover_pose = self.last_good_target

    def _tick_hover(self):
        """Wait for tracking recovery or timeout to auto-land."""
        if self.tracking_status == 'TRACKING':
            self.get_logger().info('Tracking recovered -- resuming FOLLOW')
            self.flight_state = 'FOLLOW'
            self.hover_start_time = None
            return

        if self.hover_start_time is not None:
            elapsed = (self.get_clock().now()
                       - self.hover_start_time).nanoseconds * 1e-9
            if elapsed > self.hover_timeout:
                self.get_logger().warn(
                    f'Tracking lost for {elapsed:.1f}s -- auto-landing')
                self._initiate_land()

    def _tick_land(self):
        """Monitor altitude during landing."""
        pose = self._get_active_pose()
        if pose is not None:
            if (pose.pose.position.z <= 0.02
                    and not getattr(self, '_landed_logged', False)):
                self.get_logger().info(
                    'On the ground -- disarm manually or call abort')
                self._landed_logged = True

    def _reset_to_idle(self):
        self.flight_state            = 'IDLE'
        self.hover_pose              = None
        self.land_start_time         = None
        self.land_start_alt          = None
        self._hover_logged           = False
        self._offboard_achieved      = False
        self._landed_logged          = False
        self.follow_target           = None
        self.last_good_target        = None
        self.hover_start_time        = None
        self.t265_z_auto_offset      = None
        self.t265_z_auto_offset_done = False
        # Reset pose source back to VICON for next flight (if applicable)
        self._pose_source = 'vicon' if self.use_vicon else 't265'
        self._handoff_done = not self.use_vicon
        self._stable_hover_start = None

    # == Course service handlers =============================================

    def _handle_launch(self, request, response):
        if self.flight_state not in ('IDLE', 'ABORT'):
            response.success = False
            response.message = f'Cannot launch: state={self.flight_state}'
            return response

        pose = self._get_active_pose()
        if pose is not None:
            self.hover_pose = PoseStamped()
            self.hover_pose.pose.position.x  = pose.pose.position.x
            self.hover_pose.pose.position.y  = pose.pose.position.y
            self.hover_pose.pose.position.z  = self.takeoff_altitude
            self.hover_pose.pose.orientation = pose.pose.orientation
            self.get_logger().info(
                f'Hover target: ({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f}, {self.takeoff_altitude:.2f})')
        else:
            self.get_logger().warn(
                'No pose at launch -- using default setpoint')

        self.flight_state       = 'LAUNCH'
        self.launch_time        = self.get_clock().now()
        self.last_mode_req_time = None
        self.last_arm_req_time  = None
        self._hover_logged      = False
        self._offboard_achieved = False

        self.get_logger().info(
            f'LAUNCH -- ascending to {self.takeoff_altitude}m')
        response.success = True
        response.message = f'Launching to {self.takeoff_altitude}m'
        return response

    def _handle_test(self, request, response):
        """Transition to FOLLOW mode.

        Uses the 'test' service name for course interface compatibility.
        """
        if self.flight_state not in ('LAUNCH', 'FOLLOW', 'HOVER'):
            response.success = False
            response.message = f'Cannot follow: state={self.flight_state}'
            return response

        self.flight_state = 'FOLLOW'
        self.get_logger().info('FOLLOW -- tracking target')
        response.success = True
        response.message = 'Following active'
        return response

    def _handle_land(self, request, response):
        if self.flight_state in ('IDLE', 'ABORT'):
            response.success = False
            response.message = f'Cannot land: state={self.flight_state}'
            return response

        self._initiate_land()
        response.success = True
        response.message = 'Landing initiated'
        return response

    def _initiate_land(self):
        """Begin landing sequence (used by both land command and auto-land)."""
        pose = self._get_active_pose()
        self.land_start_alt  = (pose.pose.position.z if pose
                                else self.takeoff_altitude)
        self.land_start_time = self.get_clock().now()
        self.flight_state    = 'LAND'
        self.follow_target   = None

        if pose is not None:
            if self.hover_pose is None:
                self.hover_pose = PoseStamped()
                self.hover_pose.pose.orientation = pose.pose.orientation
            self.hover_pose.pose.position.x = pose.pose.position.x
            self.hover_pose.pose.position.y = pose.pose.position.y

        self.get_logger().info(
            f'LAND -- descending from {self.land_start_alt:.2f}m '
            f'at {self.descent_speed} m/s')

    def _handle_abort(self, request, response):
        self.get_logger().warn('ABORT -- emergency disarm')
        self.flight_state       = 'ABORT'
        self.hover_pose         = None
        self.land_start_time    = None
        self.follow_target      = None
        self._offboard_achieved = False
        self._request_arm(False)

        response.success = True
        response.message = 'Emergency stop executed'
        return response

    def _handle_switch_pose(self, request, response):
        """Manual service to hand off pose source from VICON to T265."""
        if self._pose_source == 't265':
            response.success = True
            response.message = 'Already using T265'
            return response

        if self.t265_pose is None:
            response.success = False
            response.message = 'Cannot switch: T265 pose not available'
            return response

        self._do_handoff('manual service call')
        response.success = True
        response.message = (
            f'Switched to T265 (Z offset={self.t265_z_auto_offset or 0:.3f}m)')
        return response

    def _do_handoff(self, reason):
        """Switch primary pose source from VICON to T265."""
        self._pose_source = 't265'
        self._handoff_done = True
        z_off = self.t265_z_auto_offset or 0.0
        self.get_logger().info(
            f'POSE HANDOFF: VICON -> T265 ({reason})  '
            f'Z offset={z_off:.3f}m')

    # == MAVROS helpers ======================================================

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
    node = FollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
