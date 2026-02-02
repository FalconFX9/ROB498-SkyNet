#!/usr/bin/env python3
"""
Communication Node for MARY Drone
Handles low-level communication with PX4 via MAVROS.
Publishes setpoints and manages flight modes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger
import numpy as np


class CommNode(Node):
    """
    Low-level communication with PX4 flight controller.

    Subscriptions:
        /mavros/state (mavros_msgs/State): FCU state
        /mary/localization/pose (geometry_msgs/PoseStamped): Vision pose

    Publications:
        /mavros/vision_pose/pose (geometry_msgs/PoseStamped): Vision pose to FCU
        /mavros/setpoint_position/local (geometry_msgs/PoseStamped): Position setpoints

    Services:
        /mary/comm/arm (std_srvs/Trigger): Arm the drone
        /mary/comm/disarm (std_srvs/Trigger): Disarm the drone
        /mary/comm/offboard (std_srvs/Trigger): Switch to OFFBOARD mode
    """

    def __init__(self):
        super().__init__('comm_node')

        # Parameters
        self.declare_parameter('setpoint_rate', 20.0)  # Hz
        self.declare_parameter('vision_pose_rate', 30.0)  # Hz

        # State
        self.mavros_state = None
        self.vision_pose = None
        self.current_setpoint = None
        self.offboard_enabled = False

        # Initialize default setpoint (will be updated)
        self.hover_setpoint = PoseStamped()
        self.hover_setpoint.pose.position.z = 1.0  # Default hover altitude

        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_system_default
        )
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mary/localization/pose',
            self.pose_callback,
            qos_profile_system_default
        )
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            '/mary/control/setpoint',
            self.setpoint_callback,
            10
        )

        # Publishers
        self.vision_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            qos_profile_system_default
        )
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile_system_default
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Service servers
        self.arm_srv = self.create_service(
            Trigger, '/mary/comm/arm', self.handle_arm
        )
        self.disarm_srv = self.create_service(
            Trigger, '/mary/comm/disarm', self.handle_disarm
        )
        self.offboard_srv = self.create_service(
            Trigger, '/mary/comm/offboard', self.handle_offboard
        )

        # Timers
        setpoint_rate = self.get_parameter('setpoint_rate').value
        vision_rate = self.get_parameter('vision_pose_rate').value

        self.create_timer(1.0 / setpoint_rate, self.publish_setpoint)
        self.create_timer(1.0 / vision_rate, self.publish_vision_pose)

        self.get_logger().info('Comm Node initialized')
        self.get_logger().info(f'  Setpoint rate: {setpoint_rate} Hz')
        self.get_logger().info(f'  Vision pose rate: {vision_rate} Hz')

    def state_callback(self, msg: State):
        """Update MAVROS state."""
        self.mavros_state = msg

    def pose_callback(self, msg: PoseStamped):
        """Update vision pose from localization."""
        self.vision_pose = msg

        # Update hover setpoint to current position if not set
        if self.current_setpoint is None:
            self.hover_setpoint = msg

    def setpoint_callback(self, msg: PoseStamped):
        """Update position setpoint."""
        self.current_setpoint = msg

    def publish_setpoint(self):
        """Publish position setpoint at fixed rate."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        if self.current_setpoint is not None:
            msg.pose = self.current_setpoint.pose
        else:
            msg.pose = self.hover_setpoint.pose

        self.setpoint_pub.publish(msg)

    def publish_vision_pose(self):
        """Publish vision pose to MAVROS at fixed rate."""
        if self.vision_pose is None:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose = self.vision_pose.pose

        self.vision_pose_pub.publish(msg)

    # Service handlers
    def handle_arm(self, request, response):
        """Handle arm request."""
        success = self.set_arm(True)
        response.success = success
        response.message = 'Armed' if success else 'Arming failed'
        return response

    def handle_disarm(self, request, response):
        """Handle disarm request."""
        success = self.set_arm(False)
        response.success = success
        response.message = 'Disarmed' if success else 'Disarming failed'
        return response

    def handle_offboard(self, request, response):
        """Handle OFFBOARD mode request."""
        success = self.set_mode('OFFBOARD')
        response.success = success
        response.message = 'OFFBOARD mode set' if success else 'Failed to set OFFBOARD mode'
        return response

    def set_arm(self, arm: bool) -> bool:
        """Set arming state."""
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Arming service not available')
            return False

        request = CommandBool.Request()
        request.value = arm

        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().success
        return False

    def set_mode(self, mode: str) -> bool:
        """Set flight mode."""
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Set mode service not available')
            return False

        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().mode_sent
        return False


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
