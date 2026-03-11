#!/usr/bin/env python3
"""
Stationkeeping Node for ROB498 Flight Training Exercise #2.

Autonomous hover at a fixed altitude (default 0.5 m) for 30 seconds with
heading hold.  Supports VICON motion capture (Part I) and T265 visual-inertial
odometry (Part II) as pose sources.

Flight state machine:
    IDLE  ->  LAUNCH  ->  TEST  ->  LAND  |  ABORT

Course interface (service servers):
    {drone_id}/comm/launch  - Arm, OFFBOARD, ascend to test altitude
    {drone_id}/comm/test    - Begin 30 s scoring window
    {drone_id}/comm/land    - Autonomous descent and soft landing
    {drone_id}/comm/abort   - Emergency disarm

Published topics:
    /mavros/vision_pose/pose          Pose for PX4 EKF2 fusion
    /mavros/setpoint_position/local   Position + heading setpoints
    /mary/comm/flight_state           Current flight state string

Subscribed topics:
    /mavros/state                     FCU state (armed, mode)
    /mary/localization/pose           T265 VIO pose (always)
    <vicon_topic>                     VICON pose (Part I, configurable)
"""

import numpy as np
import tf_transformations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from std_srvs.srv import Trigger


class StationkeepingNode(Node):
    """Stationkeeping controller for Flight Test #2."""

    def __init__(self):
        super().__init__('stationkeeping_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('drone_id',              'rob498_drone_10')
        self.declare_parameter('takeoff_altitude',      0.5)     # m
        self.declare_parameter('setpoint_rate',         20.0)    # Hz (must be > 2 Hz for PX4)
        self.declare_parameter('vision_pose_rate',      30.0)    # Hz
        self.declare_parameter('descent_speed',         0.15)    # m/s during landing
        self.declare_parameter('land_disarm_altitude',  0.0)     # m — disarm at ground
        self.declare_parameter('offboard_wait',         1.5)     # s before OFFBOARD request
        self.declare_parameter('mode_request_interval', 2.0)     # s between OFFBOARD/arm retries
        self.declare_parameter('land_timeout',          15.0)    # s — force disarm safety net
        self.declare_parameter('vicon_topic',           '')      # VICON PoseStamped topic
        self.declare_parameter('vicon_timeout',         0.5)     # s before VICON fallback
        self.declare_parameter('t265_z_offset',         0.0)     # m — added to T265 Z (from calibration)
        self.declare_parameter('calibration_duration',  10.0)    # s — height calibration collection time
        self.declare_parameter('log_euler',             False)   # Log RPY at ~2 Hz

        drone_id                = self.get_parameter('drone_id').value
        self.takeoff_altitude   = self.get_parameter('takeoff_altitude').value
        setpoint_rate           = self.get_parameter('setpoint_rate').value
        vision_rate             = self.get_parameter('vision_pose_rate').value
        self.descent_speed      = self.get_parameter('descent_speed').value
        self.land_disarm_alt    = self.get_parameter('land_disarm_altitude').value
        self.offboard_wait      = self.get_parameter('offboard_wait').value
        self.mode_req_interval  = self.get_parameter('mode_request_interval').value
        self.land_timeout       = self.get_parameter('land_timeout').value
        vicon_topic             = self.get_parameter('vicon_topic').value
        self.vicon_timeout      = self.get_parameter('vicon_timeout').value
        self.t265_z_offset      = self.get_parameter('t265_z_offset').value
        self.calibration_dur    = self.get_parameter('calibration_duration').value
        self.log_euler          = self.get_parameter('log_euler').value
        self._euler_vicon_ctr   = 0
        self._euler_vision_ctr  = 0

        # Debug RPY publishers (created only when log_euler is enabled)
        if self.log_euler:
            self._pub_vicon_raw_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vicon_raw_rpy', 10)
            self._pub_vicon_cor_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vicon_cor_rpy', 10)
            self._pub_vision_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/vision_rpy', 10)

        # ── State ─────────────────────────────────────────────────────────
        self.flight_state       = 'IDLE'
        self.mavros_state       = None       # latest State message
        self.hover_pose         = None       # PoseStamped to hold during hover
        self.t265_pose          = None       # latest T265 pose
        self.t265_stamp         = None       # clock time of latest T265 pose
        self.vicon_pose         = None       # latest VICON pose
        self.vicon_stamp        = None       # clock time of latest VICON pose
        self.launch_time        = None       # when LAUNCH was received
        self.land_start_time    = None       # when LAND was received
        self.land_start_alt     = None       # altitude at start of landing
        self.last_mode_req_time = None       # rate-limit OFFBOARD requests
        self.last_arm_req_time  = None       # rate-limit arming requests
        self._hover_logged      = False      # one-shot log flag
        self._offboard_achieved = False      # True once armed + OFFBOARD reached
        self._raw_t265_z        = None       # pre-offset T265 Z for calibration
        self._cal_active        = False      # height calibration in progress
        self._cal_samples       = []         # list of (vicon_z, raw_t265_z)
        self._cal_start         = None       # calibration start time

        # ── Vicon body-frame orientation correction ────────────────────────
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
        self.get_logger().info('Vicon body-frame correction active')

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            State, '/mavros/state',
            self._on_mavros_state,
            qos_profile_system_default,
        )
        self.create_subscription(
            PoseStamped, '/mary/localization/pose',
            self._on_t265_pose,
            qos_profile_system_default,
        )
        if vicon_topic:
            self.create_subscription(
                PoseStamped, vicon_topic,
                self._on_vicon_pose,
                10,
            )
            self.get_logger().info(f'VICON topic: {vicon_topic}')
        else:
            self.get_logger().info('No VICON topic configured — T265 only (Part II)')

        # ── Publishers ────────────────────────────────────────────────────
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', qos_profile_system_default,
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile_system_default,
        )
        self.state_pub = self.create_publisher(String, '/mary/comm/flight_state', 10)

        # ── MAVROS service clients ────────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')

        # ── Course service servers ────────────────────────────────────────
        self.create_service(Trigger, f'{drone_id}/comm/launch',    self._handle_launch)
        self.create_service(Trigger, f'{drone_id}/comm/test',      self._handle_test)
        self.create_service(Trigger, f'{drone_id}/comm/land',      self._handle_land)
        self.create_service(Trigger, f'{drone_id}/comm/abort',     self._handle_abort)
        self.create_service(Trigger, f'{drone_id}/comm/calibrate', self._handle_calibrate)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(1.0 / setpoint_rate, self._setpoint_loop)
        self.create_timer(1.0 / vision_rate,   self._vision_pose_loop)
        self.create_timer(0.1,                 self._state_machine_loop)  # 10 Hz

        self.get_logger().info(
            f'StationkeepingNode ready  drone_id={drone_id}  '
            f'altitude={self.takeoff_altitude}m  '
            f'setpoint={setpoint_rate}Hz  vision={vision_rate}Hz'
        )

    # ── Subscriber callbacks ──────────────────────────────────────────────

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

    # ── Pose source management ────────────────────────────────────────────

    def _get_active_pose(self):
        """Return the best available pose: fresh VICON > T265 > None."""
        now = self.get_clock().now()

        # Prefer VICON if available and fresh
        if self.vicon_pose is not None and self.vicon_stamp is not None:
            age = (now - self.vicon_stamp).nanoseconds * 1e-9
            if age < self.vicon_timeout:
                return self.vicon_pose

        # Fall back to T265
        return self.t265_pose

    # ── Vision pose relay ─────────────────────────────────────────────────

    def _vision_pose_loop(self):
        """Relay the active pose to MAVROS for PX4 EKF2 fusion."""
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

    # ── Setpoint publishing ───────────────────────────────────────────────

    def _setpoint_loop(self):
        """
        Publish position setpoints at a fixed rate.

        IDLE / LAUNCH / TEST : hold hover_pose (or current pose pre-launch).
        LAND                 : maintain X/Y/heading, ramp Z toward ground.
        ABORT                : stop publishing (drone is disarmed).
        """
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.flight_state in ('IDLE', 'LAUNCH', 'TEST'):
            if self.hover_pose is not None:
                msg.pose.position.x    = self.hover_pose.pose.position.x
                msg.pose.position.y    = self.hover_pose.pose.position.y
                msg.pose.position.z    = self.hover_pose.pose.position.z
                msg.pose.orientation.x = self.hover_pose.pose.orientation.x
                msg.pose.orientation.y = self.hover_pose.pose.orientation.y
                msg.pose.orientation.z = self.hover_pose.pose.orientation.z
                msg.pose.orientation.w = self.hover_pose.pose.orientation.w
            else:
                # Pre-launch: stream current pose to keep OFFBOARD alive
                pose = self._get_active_pose()
                if pose is not None:
                    msg.pose = pose.pose
                else:
                    # No pose yet — publish safe default
                    msg.pose.position.z    = self.takeoff_altitude
                    msg.pose.orientation.w = 1.0

        elif self.flight_state == 'LAND':
            # Maintain X / Y / heading from hover, ramp Z down
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
            return  # Don't publish setpoints while aborting

        self.setpoint_pub.publish(msg)
        self._broadcast_state()

    def _broadcast_state(self):
        msg      = String()
        msg.data = self.flight_state
        self.state_pub.publish(msg)

    # ── State machine ─────────────────────────────────────────────────────

    def _state_machine_loop(self):
        """Handle state-dependent logic at 10 Hz."""

        # ── RC override / external disarm detection ──────────────────────
        # Only trigger after we've been flying in OFFBOARD, to avoid
        # false positives during the LAUNCH arming sequence.
        if (self._offboard_achieved
                and self.flight_state in ('LAUNCH', 'TEST', 'LAND')
                and self.mavros_state is not None):

            if self.mavros_state.mode != 'OFFBOARD':
                self.get_logger().info(
                    f'RC override detected (mode={self.mavros_state.mode}) '
                    f'— releasing control to pilot'
                )
                self._reset_to_idle()
                return

            if not self.mavros_state.armed:
                self.get_logger().info(
                    'External disarm detected — resetting to IDLE'
                )
                self._reset_to_idle()
                return

        if self._cal_active:
            self._tick_calibration()

        if self.flight_state == 'LAUNCH':
            self._tick_launch()
        elif self.flight_state == 'LAND':
            self._tick_land()

    def _tick_launch(self):
        """During LAUNCH: request OFFBOARD + arm after setpoints have streamed."""
        if self.mavros_state is None:
            return

        now     = self.get_clock().now()
        elapsed = (now - self.launch_time).nanoseconds * 1e-9

        # Wait for setpoint stream to build up before requesting OFFBOARD
        if elapsed < self.offboard_wait:
            return

        # Request OFFBOARD mode (rate-limited)
        if self.mavros_state.mode != 'OFFBOARD':
            if self.last_mode_req_time is None or \
               (now - self.last_mode_req_time).nanoseconds * 1e-9 > self.mode_req_interval:
                self.get_logger().info('Requesting OFFBOARD mode...')
                self._request_mode('OFFBOARD')
                self.last_mode_req_time = now
            return

        # Request arming (rate-limited)
        if not self.mavros_state.armed:
            if self.last_arm_req_time is None or \
               (now - self.last_arm_req_time).nanoseconds * 1e-9 > self.mode_req_interval:
                self.get_logger().info('Requesting arming...')
                self._request_arm(True)
                self.last_arm_req_time = now
            return

        # Armed + OFFBOARD — hovering
        self._offboard_achieved = True
        if not self._hover_logged:
            self.get_logger().info('Armed in OFFBOARD — hovering, ready for TEST')
            self._hover_logged = True

    def _tick_land(self):
        """During LAND: ramp Z setpoint to ground. Manual disarm required."""
        pose = self._get_active_pose()
        if pose is not None:
            alt = pose.pose.position.z
            if alt <= 0.02 and not getattr(self, '_landed_logged', False):
                self.get_logger().info('On the ground — disarm manually or call abort')
                self._landed_logged = True

    def _reset_to_idle(self):
        """Clean up state and return to IDLE."""
        self.flight_state       = 'IDLE'
        self.hover_pose         = None
        self.land_start_time    = None
        self.land_start_alt     = None
        self._hover_logged      = False
        self._offboard_achieved = False
        self._landed_logged     = False

    # ── Course service handlers ───────────────────────────────────────────

    def _handle_launch(self, request, response):
        """
        Handle LAUNCH command from ground control.

        Captures the hover reference pose (current X/Y + target Z + current
        heading) and begins streaming setpoints.  The state machine will
        request OFFBOARD mode and arming asynchronously.
        """
        if self.flight_state not in ('IDLE', 'ABORT'):
            response.success = False
            response.message = f'Cannot launch: state={self.flight_state}'
            return response

        # Capture hover pose from best available sensor
        pose = self._get_active_pose()
        if pose is not None:
            self.hover_pose = PoseStamped()
            self.hover_pose.pose.position.x    = pose.pose.position.x
            self.hover_pose.pose.position.y    = pose.pose.position.y
            self.hover_pose.pose.position.z    = self.takeoff_altitude
            self.hover_pose.pose.orientation.x = pose.pose.orientation.x
            self.hover_pose.pose.orientation.y = pose.pose.orientation.y
            self.hover_pose.pose.orientation.z = pose.pose.orientation.z
            self.hover_pose.pose.orientation.w = pose.pose.orientation.w
            self.get_logger().info(
                f'Hover target: ({pose.pose.position.x:.2f}, '
                f'{pose.pose.position.y:.2f}, {self.takeoff_altitude:.2f})'
            )
        else:
            self.get_logger().warn(
                'No pose available at launch — using default setpoint'
            )

        self.flight_state       = 'LAUNCH'
        self.launch_time        = self.get_clock().now()
        self.last_mode_req_time = None
        self.last_arm_req_time  = None
        self._hover_logged      = False
        self._offboard_achieved = False

        self.get_logger().info(
            f'LAUNCH received — ascending to {self.takeoff_altitude}m'
        )
        response.success = True
        response.message = f'Launching to {self.takeoff_altitude}m'
        return response

    def _handle_test(self, request, response):
        """
        Handle TEST command from ground control.

        Marks the start of the 30 s scoring window.  The drone continues
        holding its hover pose.  The reference pose for scoring is wherever
        the drone is when this command arrives.
        """
        if self.flight_state not in ('LAUNCH', 'TEST'):
            response.success = False
            response.message = f'Cannot test: state={self.flight_state}'
            return response

        pose = self._get_active_pose()
        if pose is not None:
            p = pose.pose.position
            self.get_logger().info(
                f'TEST started — reference pose: '
                f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
            )
        else:
            self.get_logger().info('TEST started — no pose available for reference log')

        self.flight_state = 'TEST'
        response.success  = True
        response.message  = 'Scoring window active'
        return response

    def _handle_land(self, request, response):
        """
        Handle LAND command from ground control.

        Begins a smooth descent at descent_speed m/s while maintaining X/Y
        position and heading.  Disarms when near the ground.
        """
        if self.flight_state in ('IDLE', 'ABORT'):
            response.success = False
            response.message = f'Cannot land: state={self.flight_state}'
            return response

        pose = self._get_active_pose()
        self.land_start_alt  = pose.pose.position.z if pose else self.takeoff_altitude
        self.land_start_time = self.get_clock().now()
        self.flight_state    = 'LAND'

        self.get_logger().info(
            f'LAND received — descending from {self.land_start_alt:.2f}m '
            f'at {self.descent_speed} m/s'
        )
        response.success = True
        response.message = 'Landing initiated'
        return response

    def _handle_abort(self, request, response):
        """
        Handle ABORT command from ground control.

        Immediately disarms the drone regardless of current state.
        """
        self.get_logger().warn('ABORT — emergency disarm')
        self.flight_state       = 'ABORT'
        self.hover_pose         = None
        self.land_start_time    = None
        self._offboard_achieved = False
        self._request_arm(False)

        response.success = True
        response.message = 'Emergency stop executed'
        return response

    def _handle_calibrate(self, request, response):
        """
        Handle CALIBRATE command.

        Collects simultaneous VICON and T265 Z samples for calibration_duration
        seconds, then computes the median offset to align T265 height with VICON.
        Must be run with Part 1 (VICON active).
        """
        if self.vicon_pose is None:
            response.success = False
            response.message = 'No VICON data — run with part:=1 for calibration'
            return response
        if self.t265_pose is None:
            response.success = False
            response.message = 'No T265 data available'
            return response

        self._cal_active  = True
        self._cal_samples = []
        self._cal_start   = self.get_clock().now()

        self.get_logger().info(
            f'Height calibration started — collecting for {self.calibration_dur}s'
        )
        response.success = True
        response.message = f'Collecting samples for {self.calibration_dur}s'
        return response

    # ── Height calibration ───────────────────────────────────────────────

    def _tick_calibration(self):
        """Collect paired VICON/T265 Z samples at 10 Hz."""
        now     = self.get_clock().now()
        elapsed = (now - self._cal_start).nanoseconds * 1e-9

        if elapsed >= self.calibration_dur:
            self._finish_calibration()
            return

        # Both sources must be fresh
        if (self.vicon_pose is not None and self.vicon_stamp is not None
                and self._raw_t265_z is not None and self.t265_stamp is not None):
            vicon_age = (now - self.vicon_stamp).nanoseconds * 1e-9
            t265_age  = (now - self.t265_stamp).nanoseconds * 1e-9

            if vicon_age < self.vicon_timeout and t265_age < 0.5:
                vicon_z = self.vicon_pose.pose.position.z
                self._cal_samples.append((vicon_z, self._raw_t265_z))

    def _finish_calibration(self):
        """Compute and apply the median Z offset."""
        self._cal_active = False
        n = len(self._cal_samples)

        if n < 10:
            self.get_logger().warn(
                f'Calibration failed — only {n} samples (need >= 10). '
                f'Ensure both VICON and T265 are streaming.'
            )
            return

        vicon_zs = np.array([s[0] for s in self._cal_samples])
        t265_zs  = np.array([s[1] for s in self._cal_samples])
        offsets  = vicon_zs - t265_zs

        median_off = float(np.median(offsets))
        mean_off   = float(np.mean(offsets))
        std_off    = float(np.std(offsets))

        self.get_logger().info('=' * 60)
        self.get_logger().info('HEIGHT CALIBRATION COMPLETE')
        self.get_logger().info(f'  Samples:       {n}')
        self.get_logger().info(f'  Median offset: {median_off:+.4f} m')
        self.get_logger().info(f'  Mean offset:   {mean_off:+.4f} m')
        self.get_logger().info(f'  Std dev:       {std_off:.4f} m')
        self.get_logger().info(f'  VICON Z  avg:  {float(np.mean(vicon_zs)):.4f} m')
        self.get_logger().info(f'  T265 Z   avg:  {float(np.mean(t265_zs)):.4f} m')
        self.get_logger().info(f'')
        self.get_logger().info(f'  Use in a future launch:')
        self.get_logger().info(f'    t265_z_offset:={median_off:.4f}')
        self.get_logger().info('=' * 60)

    # ── MAVROS helpers ────────────────────────────────────────────────────

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
    node = StationkeepingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
