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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import PoseStamped
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
        self.declare_parameter('takeoff_altitude',      0.5)     # m  (Exercise #2: 50 cm)
        self.declare_parameter('setpoint_rate',         20.0)    # Hz (must be > 2 Hz for PX4)
        self.declare_parameter('vision_pose_rate',      30.0)    # Hz
        self.declare_parameter('descent_speed',         0.15)    # m/s during landing
        self.declare_parameter('land_disarm_altitude',  0.12)    # m — disarm below this
        self.declare_parameter('offboard_wait',         1.5)     # s before OFFBOARD request
        self.declare_parameter('mode_request_interval', 2.0)     # s between OFFBOARD/arm retries
        self.declare_parameter('land_timeout',          15.0)    # s — force disarm safety net
        self.declare_parameter('vicon_topic',           '')      # VICON PoseStamped topic
        self.declare_parameter('vicon_timeout',         0.5)     # s before VICON fallback

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
        self.create_service(Trigger, f'{drone_id}/comm/launch', self._handle_launch)
        self.create_service(Trigger, f'{drone_id}/comm/test',   self._handle_test)
        self.create_service(Trigger, f'{drone_id}/comm/land',   self._handle_land)
        self.create_service(Trigger, f'{drone_id}/comm/abort',  self._handle_abort)

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
        self.t265_pose  = msg
        self.t265_stamp = self.get_clock().now()

    def _on_vicon_pose(self, msg):
        self.vicon_pose  = msg
        self.vicon_stamp = self.get_clock().now()

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
                msg.pose.position.z = max(target_z, 0.03)
            else:
                msg.pose.position.z = 0.03

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
        """During LAND: monitor altitude and disarm when near ground."""
        now     = self.get_clock().now()
        elapsed = (now - self.land_start_time).nanoseconds * 1e-9

        pose = self._get_active_pose()
        if pose is not None:
            alt = pose.pose.position.z
            if alt < self.land_disarm_alt:
                self.get_logger().info(
                    f'Altitude {alt:.2f}m < {self.land_disarm_alt}m — disarming'
                )
                self._request_arm(False)
                self._reset_to_idle()
                return

        # Safety timeout: force disarm if landing takes too long
        if elapsed > self.land_timeout:
            self.get_logger().warn('Landing timeout — forcing disarm')
            self._request_arm(False)
            self._reset_to_idle()

    def _reset_to_idle(self):
        """Clean up state and return to IDLE."""
        self.flight_state       = 'IDLE'
        self.hover_pose         = None
        self.land_start_time    = None
        self.land_start_alt     = None
        self._hover_logged      = False
        self._offboard_achieved = False

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
