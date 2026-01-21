#!/usr/bin/env python3
"""
Mission Manager Node for MARY Drone
High-level state machine for mission control:
- Startup/calibration
- Takeoff
- Person acquisition
- Following
- Person switching
- Landing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import numpy as np
from enum import Enum, auto


class MissionState(Enum):
    """Mission state machine states."""
    IDLE = auto()
    PREFLIGHT = auto()
    ARMING = auto()
    TAKEOFF = auto()
    ACQUIRING = auto()  # Searching for person
    FOLLOWING = auto()
    SWITCHING = auto()  # Switching to new person
    LANDING = auto()
    EMERGENCY = auto()


class MissionManagerNode(Node):
    """
    High-level mission manager for MARY drone.

    Services:
        /mary/mission/start (std_srvs/Trigger): Start mission
        /mary/mission/stop (std_srvs/Trigger): Stop mission and land
        /mary/mission/switch_person (std_srvs/Trigger): Switch to new person
        /mary/mission/emergency_stop (std_srvs/Trigger): Emergency stop

    Publications:
        /mary/mission/state (std_msgs/String): Current mission state
        /mary/mission/status (std_msgs/String): Human-readable status
    """

    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('takeoff_altitude', 2.5)   # meters
        self.declare_parameter('arming_timeout', 10.0)    # seconds
        self.declare_parameter('takeoff_timeout', 30.0)   # seconds
        self.declare_parameter('acquire_timeout', 60.0)   # seconds

        # Get parameters
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.arming_timeout = self.get_parameter('arming_timeout').value

        # State
        self.state = MissionState.IDLE
        self.mavros_state = None
        self.current_altitude = None
        self.person_detected = False
        self.drone_pose = None
        self.state_entry_time = None

        # MAVROS state subscriber
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.mavros_state_callback,
            qos_profile_system_default
        )

        # Other subscribers
        self.altitude_sub = self.create_subscription(
            Float32,
            '/mary/perception/altitude',
            self.altitude_callback,
            10
        )
        self.detected_sub = self.create_subscription(
            Bool,
            '/mary/perception/person_detected',
            self.person_detected_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mary/localization/pose',
            self.pose_callback,
            qos_profile_system_default
        )

        # Publishers
        self.state_pub = self.create_publisher(String, '/mary/mission/state', 10)
        self.status_pub = self.create_publisher(String, '/mary/mission/status', 10)

        # Service clients for MAVROS
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Service servers
        self.start_srv = self.create_service(
            Trigger, '/mary/mission/start', self.handle_start
        )
        self.stop_srv = self.create_service(
            Trigger, '/mary/mission/stop', self.handle_stop
        )
        self.switch_srv = self.create_service(
            Trigger, '/mary/mission/switch_person', self.handle_switch
        )
        self.emergency_srv = self.create_service(
            Trigger, '/mary/mission/emergency_stop', self.handle_emergency
        )

        # State machine timer
        self.create_timer(0.1, self.state_machine_update)  # 10 Hz

        self.get_logger().info('Mission Manager Node initialized')
        self.get_logger().info(f'  Takeoff altitude: {self.takeoff_altitude}m')

    def mavros_state_callback(self, msg: State):
        """Update MAVROS state."""
        self.mavros_state = msg

    def altitude_callback(self, msg: Float32):
        """Update current altitude."""
        self.current_altitude = msg.data

    def person_detected_callback(self, msg: Bool):
        """Update person detection status."""
        self.person_detected = msg.data

    def pose_callback(self, msg: PoseStamped):
        """Update drone pose."""
        self.drone_pose = msg

    def transition_to(self, new_state: MissionState):
        """Transition to a new state."""
        old_state = self.state
        self.state = new_state
        self.state_entry_time = self.get_clock().now()

        self.get_logger().info(f'State transition: {old_state.name} -> {new_state.name}')

        # Publish state
        state_msg = String()
        state_msg.data = new_state.name
        self.state_pub.publish(state_msg)

    def state_machine_update(self):
        """Main state machine update loop."""
        if self.state == MissionState.IDLE:
            self.handle_idle_state()
        elif self.state == MissionState.PREFLIGHT:
            self.handle_preflight_state()
        elif self.state == MissionState.ARMING:
            self.handle_arming_state()
        elif self.state == MissionState.TAKEOFF:
            self.handle_takeoff_state()
        elif self.state == MissionState.ACQUIRING:
            self.handle_acquiring_state()
        elif self.state == MissionState.FOLLOWING:
            self.handle_following_state()
        elif self.state == MissionState.SWITCHING:
            self.handle_switching_state()
        elif self.state == MissionState.LANDING:
            self.handle_landing_state()
        elif self.state == MissionState.EMERGENCY:
            self.handle_emergency_state()

    def handle_idle_state(self):
        """Handle IDLE state - waiting for start command."""
        pass  # Just wait

    def handle_preflight_state(self):
        """Handle PREFLIGHT state - check systems."""
        # Check if MAVROS is connected
        if self.mavros_state is None:
            self.publish_status('Waiting for MAVROS connection...')
            return

        if not self.mavros_state.connected:
            self.publish_status('Waiting for FCU connection...')
            return

        # Check if pose is available
        if self.drone_pose is None:
            self.publish_status('Waiting for localization...')
            return

        self.publish_status('Preflight checks passed')
        self.transition_to(MissionState.ARMING)

    def handle_arming_state(self):
        """Handle ARMING state - arm the drone."""
        if self.mavros_state is None:
            return

        # Check timeout
        elapsed = (self.get_clock().now() - self.state_entry_time).nanoseconds * 1e-9
        if elapsed > self.arming_timeout:
            self.publish_status('Arming timeout!')
            self.transition_to(MissionState.IDLE)
            return

        # Request OFFBOARD mode
        if self.mavros_state.mode != 'OFFBOARD':
            self.request_mode('OFFBOARD')
            return

        # Request arming
        if not self.mavros_state.armed:
            self.request_arming(True)
            return

        self.publish_status('Armed successfully')
        self.transition_to(MissionState.TAKEOFF)

    def handle_takeoff_state(self):
        """Handle TAKEOFF state - ascend to target altitude."""
        if self.current_altitude is None:
            return

        # Check if reached target altitude
        if self.current_altitude >= self.takeoff_altitude * 0.95:
            self.publish_status(f'Takeoff complete at {self.current_altitude:.1f}m')
            self.transition_to(MissionState.ACQUIRING)
        else:
            self.publish_status(f'Taking off... {self.current_altitude:.1f}/{self.takeoff_altitude:.1f}m')

    def handle_acquiring_state(self):
        """Handle ACQUIRING state - search for person."""
        if self.person_detected:
            self.publish_status('Person acquired!')
            self.transition_to(MissionState.FOLLOWING)
        else:
            self.publish_status('Searching for person...')

    def handle_following_state(self):
        """Handle FOLLOWING state - follow person."""
        if not self.person_detected:
            self.publish_status('Lost person - searching...')
            # Don't immediately transition - follower_node handles brief losses
        else:
            self.publish_status('Following person')

    def handle_switching_state(self):
        """Handle SWITCHING state - switch to new person."""
        # TODO: Implement person switching logic
        self.publish_status('Switching person...')
        self.transition_to(MissionState.ACQUIRING)

    def handle_landing_state(self):
        """Handle LANDING state - descend and disarm."""
        if self.current_altitude is not None and self.current_altitude < 0.3:
            # Near ground - disarm
            self.request_arming(False)
            self.publish_status('Landed')
            self.transition_to(MissionState.IDLE)
        else:
            self.publish_status(f'Landing... {self.current_altitude:.1f}m')

    def handle_emergency_state(self):
        """Handle EMERGENCY state."""
        # Request disarm
        self.request_arming(False)
        self.publish_status('EMERGENCY STOP')

    # Service handlers
    def handle_start(self, request, response):
        """Handle start mission request."""
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f'Cannot start - current state: {self.state.name}'
        else:
            self.transition_to(MissionState.PREFLIGHT)
            response.success = True
            response.message = 'Mission starting'
        return response

    def handle_stop(self, request, response):
        """Handle stop mission request."""
        self.transition_to(MissionState.LANDING)
        response.success = True
        response.message = 'Landing initiated'
        return response

    def handle_switch(self, request, response):
        """Handle switch person request."""
        if self.state == MissionState.FOLLOWING:
            self.transition_to(MissionState.SWITCHING)
            response.success = True
            response.message = 'Switching to new person'
        else:
            response.success = False
            response.message = f'Cannot switch - current state: {self.state.name}'
        return response

    def handle_emergency(self, request, response):
        """Handle emergency stop request."""
        self.transition_to(MissionState.EMERGENCY)
        response.success = True
        response.message = 'Emergency stop activated'
        return response

    # Helper methods
    def publish_status(self, status: str):
        """Publish human-readable status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def request_mode(self, mode: str):
        """Request flight mode change."""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Set mode service not available')
            return

        request = SetMode.Request()
        request.custom_mode = mode
        future = self.set_mode_client.call_async(request)

    def request_arming(self, arm: bool):
        """Request arming/disarming."""
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Arming service not available')
            return

        request = CommandBool.Request()
        request.value = arm
        future = self.arming_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
