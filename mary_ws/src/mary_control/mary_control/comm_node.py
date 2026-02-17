#!/usr/bin/env python3
"""
Communication Node for MARY Drone.

Implements the ROB498 course communication interface and manages
vision pose relay and position setpoints for MAVROS.

Course interface (services + subscriber):
    {drone_id}/comm/launch     (Trigger) - Arm and ascend to takeoff altitude
    {drone_id}/comm/test       (Trigger) - Begin test/scoring window
    {drone_id}/comm/land       (Trigger) - Graceful landing
    {drone_id}/comm/abort      (Trigger) - Emergency stop
    {drone_id}/comm/waypoints  (PoseArray) - Receive waypoints (Exercise #3)

Internal services (for manual testing / other nodes):
    /mary/comm/arm      (Trigger) - Arm directly
    /mary/comm/disarm   (Trigger) - Disarm directly
    /mary/comm/offboard (Trigger) - Switch to OFFBOARD mode directly

Flight state machine:
    IDLE -> LAUNCH -> TEST -> LAND | ABORT
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from geometry_msgs.msg import PoseStamped, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CommNode(Node):
    """
    Communication node implementing the ROB498 course interface.

    On launch:  arms the drone, switches to OFFBOARD mode, and delegates
                to the mission manager to execute autonomous takeoff.
    On test:    marks the start of the scoring window; the mission manager
                continues running (hovering or following a person).
    On land:    commands AUTO.LAND and notifies the mission manager.
    On abort:   immediately disarms; no graceful shutdown.

    Vision pose from /mary/localization/pose is relayed to MAVROS at a
    fixed rate so PX4 EKF2 has a continuous pose stream.

    Position setpoints are published at a fixed rate to keep OFFBOARD mode
    alive and to hold the hover position when no higher-level controller is
    active.
    """

    def __init__(self):
        super().__init__('comm_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('drone_id',          'rob498_drone_07')
        self.declare_parameter('takeoff_altitude',  1.5)   # meters (Exercise #2)
        self.declare_parameter('setpoint_rate',     20.0)  # Hz
        self.declare_parameter('vision_pose_rate',  30.0)  # Hz

        drone_id               = self.get_parameter('drone_id').value
        self.takeoff_altitude  = self.get_parameter('takeoff_altitude').value
        setpoint_rate          = self.get_parameter('setpoint_rate').value
        vision_rate            = self.get_parameter('vision_pose_rate').value

        # ── State ─────────────────────────────────────────────────────────────
        # Flight state: IDLE | LAUNCH | TEST | LAND | ABORT
        self.flight_state = 'IDLE'

        self.mavros_state = None
        self.vision_pose  = None
        # hover_pose is captured at launch time: current X/Y + takeoff altitude.
        # It is the position setpoint used during IDLE/LAUNCH/TEST states.
        self.hover_pose   = None

        # Waypoints received from ground station (Exercise #3), shape (N, 3).
        self.waypoints          = None
        self.waypoints_received = False

        # True after mission manager has been successfully started, so we know
        # the follower node may be publishing velocity setpoints.
        self.mission_manager_active = False

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            State,
            '/mavros/state',
            self._on_mavros_state,
            qos_profile_system_default,
        )
        self.create_subscription(
            PoseStamped,
            '/mary/localization/pose',
            self._on_pose,
            qos_profile_system_default,
        )
        # Waypoints for Exercise #3
        self.create_subscription(
            PoseArray,
            f'{drone_id}/comm/waypoints',
            self._on_waypoints,
            10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.vision_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', qos_profile_system_default,
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos_profile_system_default,
        )
        # Broadcast current flight state for other nodes to observe
        self.flight_state_pub = self.create_publisher(String, '/mary/comm/flight_state', 10)

        # ── MAVROS service clients ─────────────────────────────────────────────
        self.arming_client   = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode,     '/mavros/set_mode')

        # ── Mission manager service clients (optional — graceful if absent) ────
        self.mission_start_client     = self.create_client(Trigger, '/mary/mission/start')
        self.mission_stop_client      = self.create_client(Trigger, '/mary/mission/stop')
        self.mission_emergency_client = self.create_client(Trigger, '/mary/mission/emergency_stop')

        # ── Course-required service servers ───────────────────────────────────
        self.create_service(Trigger, f'{drone_id}/comm/launch', self._handle_launch)
        self.create_service(Trigger, f'{drone_id}/comm/test',   self._handle_test)
        self.create_service(Trigger, f'{drone_id}/comm/land',   self._handle_land)
        self.create_service(Trigger, f'{drone_id}/comm/abort',  self._handle_abort)

        # ── Internal convenience services ─────────────────────────────────────
        self.create_service(Trigger, '/mary/comm/arm',      self._handle_arm)
        self.create_service(Trigger, '/mary/comm/disarm',   self._handle_disarm)
        self.create_service(Trigger, '/mary/comm/offboard', self._handle_offboard)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / setpoint_rate, self._publish_setpoint)
        self.create_timer(1.0 / vision_rate,   self._publish_vision_pose)

        self.get_logger().info(
            f'CommNode ready  drone_id={drone_id}  '
            f'takeoff={self.takeoff_altitude} m  '
            f'setpoint={setpoint_rate} Hz  vision={vision_rate} Hz'
        )

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _on_mavros_state(self, msg: State):
        self.mavros_state = msg

    def _on_pose(self, msg: PoseStamped):
        self.vision_pose = msg

    def _on_waypoints(self, msg: PoseArray):
        """Store waypoints received from the ground station (Exercise #3)."""
        if self.waypoints_received:
            return
        self.waypoints_received = True
        pts = [[p.position.x, p.position.y, p.position.z] for p in msg.poses]
        self.waypoints = np.array(pts) if pts else np.empty((0, 3))
        self.get_logger().info(f'Received {len(self.waypoints)} waypoints')

    # ── Publisher timers ──────────────────────────────────────────────────────

    def _publish_setpoint(self):
        """
        Publish position setpoints at a fixed rate.

        IDLE/LAUNCH/TEST: hold hover_pose (captured at launch time).
        LAND:             descend to z=0 at current X/Y.
        ABORT:            no setpoints published (drone is disarmed).

        During TEST state with the mission manager active the follower node
        publishes velocity setpoints directly to MAVROS.  We keep publishing
        the hover position here so OFFBOARD mode stays alive during brief
        detection losses; PX4 uses the most-recently-received setpoint so the
        follower's velocity commands take precedence when it is running.
        """
        msg = PoseStamped()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'

        if self.flight_state in ('IDLE', 'LAUNCH', 'TEST'):
            if self.hover_pose is not None:
                msg.pose = self.hover_pose.pose
            elif self.vision_pose is not None:
                # Pre-launch: hold current position as a no-op stream
                msg.pose = self.vision_pose.pose
            else:
                msg.pose.position.z = self.takeoff_altitude

        elif self.flight_state == 'LAND':
            # AUTO.LAND handles the actual descent; publish a ground-level
            # position as a fallback in case the mode switch was rejected.
            if self.vision_pose is not None:
                msg.pose.position.x = self.vision_pose.pose.position.x
                msg.pose.position.y = self.vision_pose.pose.position.y
            msg.pose.position.z = 0.0

        else:  # ABORT — do not publish setpoints
            self._broadcast_state()
            return

        self.setpoint_pub.publish(msg)
        self._broadcast_state()

    def _publish_vision_pose(self):
        """Relay T265 pose to MAVROS so PX4 EKF2 has a continuous pose stream."""
        if self.vision_pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose            = self.vision_pose.pose
        self.vision_pose_pub.publish(msg)

    def _broadcast_state(self):
        state_msg      = String()
        state_msg.data = self.flight_state
        self.flight_state_pub.publish(state_msg)

    # ── Course service handlers ───────────────────────────────────────────────

    def _handle_launch(self, request, response):
        """
        Arm the drone, switch to OFFBOARD, and begin autonomous takeoff.

        Captures the hover position from the current vision pose so the drone
        holds its X/Y position while climbing to takeoff_altitude.

        Delegates arming + takeoff sequencing to the mission manager when
        available.  Falls back to direct MAVROS calls otherwise.
        """
        if self.flight_state != 'IDLE':
            response.success = False
            response.message = f'Rejected: already in state {self.flight_state}'
            return response

        self.get_logger().info('Launch requested')

        # Capture hover origin from current pose
        if self.vision_pose is not None:
            self.hover_pose = PoseStamped()
            self.hover_pose.pose.position.x   = self.vision_pose.pose.position.x
            self.hover_pose.pose.position.y   = self.vision_pose.pose.position.y
            self.hover_pose.pose.position.z   = self.takeoff_altitude
            self.hover_pose.pose.orientation  = self.vision_pose.pose.orientation
        else:
            self.get_logger().warn('No pose available at launch — will use default setpoint')

        # Try delegating to mission manager
        if self.mission_start_client.wait_for_service(timeout_sec=1.0):
            fut = self.mission_start_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            result = fut.result()
            if result is not None and result.success:
                self.flight_state           = 'LAUNCH'
                self.mission_manager_active = True
                response.success            = True
                response.message            = f'Launch initiated (mission manager: {result.message})'
                return response
            else:
                msg = result.message if result else 'timeout'
                self.get_logger().warn(f'Mission manager rejected launch ({msg}) — falling back')

        # Standalone fallback: arm + OFFBOARD directly
        if not self._set_mode('OFFBOARD'):
            response.success = False
            response.message = 'Failed to set OFFBOARD mode'
            return response
        if not self._set_arm(True):
            response.success = False
            response.message = 'Arming failed'
            return response

        self.flight_state = 'LAUNCH'
        response.success  = True
        response.message  = 'Launch initiated (standalone — no mission manager)'
        return response

    def _handle_test(self, request, response):
        """
        Mark the start of the test/scoring window.

        The mission manager is assumed to already be running (started by
        /comm/launch).  For Exercise #2 the drone continues hovering;
        for the MARY mission it is already in ACQUIRING/FOLLOWING state.
        """
        self.get_logger().info('Test requested — scoring window started')
        self.flight_state = 'TEST'
        response.success  = True
        response.message  = 'Test sequence active'
        return response

    def _handle_land(self, request, response):
        """
        Initiate a graceful landing via PX4 AUTO.LAND mode.

        Notifies the mission manager so it can stop person-following cleanly.
        """
        self.get_logger().info('Land requested')
        prev_state        = self.flight_state
        self.flight_state = 'LAND'

        # Notify mission manager if it was active
        if self.mission_manager_active:
            self.mission_manager_active = False
            if self.mission_stop_client.wait_for_service(timeout_sec=1.0):
                self.mission_stop_client.call_async(Trigger.Request())

        # Switch to AUTO.LAND — PX4 handles the descent autonomously
        if not self._set_mode('AUTO.LAND'):
            self.get_logger().warn('AUTO.LAND mode rejected — publishing z=0 setpoints as fallback')

        response.success = True
        response.message = 'Landing initiated'
        return response

    def _handle_abort(self, request, response):
        """
        Emergency stop: immediately disarm the drone.

        Notifies the mission manager on a best-effort basis, then forces
        disarm regardless of current state.
        """
        self.get_logger().warn('ABORT — emergency disarm')
        self.flight_state           = 'ABORT'
        self.mission_manager_active = False

        # Best-effort notification to mission manager
        if self.mission_emergency_client.wait_for_service(timeout_sec=0.2):
            self.mission_emergency_client.call_async(Trigger.Request())

        # Force disarm (synchronous — safety critical)
        self._set_arm(False)

        response.success = True
        response.message = 'Emergency stop executed'
        return response

    # ── Internal convenience service handlers ─────────────────────────────────

    def _handle_arm(self, request, response):
        ok               = self._set_arm(True)
        response.success = ok
        response.message = 'Armed' if ok else 'Arming failed'
        return response

    def _handle_disarm(self, request, response):
        ok               = self._set_arm(False)
        response.success = ok
        response.message = 'Disarmed' if ok else 'Disarming failed'
        return response

    def _handle_offboard(self, request, response):
        ok               = self._set_mode('OFFBOARD')
        response.success = ok
        response.message = 'OFFBOARD mode set' if ok else 'Failed to set OFFBOARD mode'
        return response

    # ── MAVROS helpers ────────────────────────────────────────────────────────

    def _set_arm(self, arm: bool) -> bool:
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service unavailable')
            return False
        req       = CommandBool.Request()
        req.value = arm
        fut       = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().success)

    def _set_mode(self, mode: str) -> bool:
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set mode service unavailable')
            return False
        req             = SetMode.Request()
        req.custom_mode = mode
        fut             = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.result() and fut.result().mode_sent)


def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
