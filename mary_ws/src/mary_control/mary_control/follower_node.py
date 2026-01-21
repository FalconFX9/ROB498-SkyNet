#!/usr/bin/env python3
"""
Person Follower Node for MARY Drone
Generates velocity commands to follow a detected person while maintaining
the umbrella position above their head.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
import numpy as np


class FollowerNode(Node):
    """
    Controls drone to follow a detected person.

    Subscriptions:
        /mary/perception/person_position (geometry_msgs/PointStamped): Person position in camera frame
        /mary/perception/person_velocity (geometry_msgs/Vector3Stamped): Person velocity estimate
        /mary/perception/person_detected (std_msgs/Bool): Detection status
        /mary/localization/pose (geometry_msgs/PoseStamped): Current drone pose
        /mary/perception/altitude (std_msgs/Float32): Current altitude above person

    Publications:
        /mavros/setpoint_velocity/cmd_vel (geometry_msgs/TwistStamped): Velocity commands
        /mary/control/target_position (geometry_msgs/PoseStamped): Target position for debugging
    """

    # Control modes
    MODE_IDLE = 0
    MODE_SEARCH = 1
    MODE_FOLLOW = 2
    MODE_HOVER = 3

    def __init__(self):
        super().__init__('follower_node')

        # Parameters
        self.declare_parameter('target_altitude', 2.5)       # meters above person
        self.declare_parameter('follow_distance', 0.0)       # horizontal offset (0 = directly above)
        self.declare_parameter('max_horizontal_speed', 2.0)  # m/s
        self.declare_parameter('max_vertical_speed', 1.0)    # m/s
        self.declare_parameter('position_p_gain', 1.0)       # Proportional gain
        self.declare_parameter('velocity_ff_gain', 0.8)      # Velocity feedforward gain
        self.declare_parameter('detection_timeout', 2.0)     # seconds before switching to search
        self.declare_parameter('control_rate', 20.0)         # Hz

        # Get parameters
        self.target_altitude = self.get_parameter('target_altitude').value
        self.max_h_speed = self.get_parameter('max_horizontal_speed').value
        self.max_v_speed = self.get_parameter('max_vertical_speed').value
        self.p_gain = self.get_parameter('position_p_gain').value
        self.ff_gain = self.get_parameter('velocity_ff_gain').value
        self.detection_timeout = self.get_parameter('detection_timeout').value
        self.control_rate = self.get_parameter('control_rate').value

        # State variables
        self.mode = self.MODE_IDLE
        self.person_detected = False
        self.last_detection_time = None
        self.person_position = None      # In camera/world frame
        self.person_velocity = None      # Estimated velocity
        self.drone_pose = None           # Current drone pose
        self.current_altitude = None     # ToF altitude

        # Subscribers
        self.person_pos_sub = self.create_subscription(
            PointStamped,
            '/mary/perception/person_position',
            self.person_position_callback,
            10
        )
        self.person_vel_sub = self.create_subscription(
            Vector3Stamped,
            '/mary/perception/person_velocity',
            self.person_velocity_callback,
            10
        )
        self.person_detected_sub = self.create_subscription(
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
        self.altitude_sub = self.create_subscription(
            Float32,
            '/mary/perception/altitude',
            self.altitude_callback,
            10
        )

        # Publishers
        self.vel_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            qos_profile_system_default
        )
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/mary/control/target_position',
            10
        )

        # Control timer
        self.create_timer(1.0 / self.control_rate, self.control_loop)

        self.get_logger().info('Follower Node initialized')
        self.get_logger().info(f'  Target altitude: {self.target_altitude}m')
        self.get_logger().info(f'  Max speeds: H={self.max_h_speed}m/s, V={self.max_v_speed}m/s')

    def person_position_callback(self, msg: PointStamped):
        """Update person position."""
        self.person_position = np.array([msg.point.x, msg.point.y, msg.point.z])

    def person_velocity_callback(self, msg: Vector3Stamped):
        """Update person velocity estimate."""
        self.person_velocity = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def person_detected_callback(self, msg: Bool):
        """Update detection status."""
        self.person_detected = msg.data
        if self.person_detected:
            self.last_detection_time = self.get_clock().now()

    def pose_callback(self, msg: PoseStamped):
        """Update current drone pose."""
        self.drone_pose = msg

    def altitude_callback(self, msg: Float32):
        """Update current altitude from ToF."""
        self.current_altitude = msg.data

    def control_loop(self):
        """Main control loop - runs at control_rate Hz."""
        # Update mode based on detection status
        self.update_mode()

        # Generate velocity command based on mode
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = 'base_link'

        if self.mode == self.MODE_FOLLOW:
            vel_cmd = self.compute_follow_velocity()
        elif self.mode == self.MODE_SEARCH:
            vel_cmd = self.compute_search_velocity()
        elif self.mode == self.MODE_HOVER:
            vel_cmd = self.compute_hover_velocity()
        # MODE_IDLE: zero velocity (already initialized)

        self.vel_pub.publish(vel_cmd)

    def update_mode(self):
        """Update control mode based on system state."""
        current_time = self.get_clock().now()

        if self.mode == self.MODE_IDLE:
            # Stay idle until explicitly activated
            pass

        elif self.person_detected:
            self.mode = self.MODE_FOLLOW

        elif self.last_detection_time is not None:
            # Check for detection timeout
            time_since_detection = (current_time - self.last_detection_time).nanoseconds * 1e-9
            if time_since_detection > self.detection_timeout:
                self.mode = self.MODE_SEARCH
            else:
                self.mode = self.MODE_HOVER  # Brief loss - hover in place

    def compute_follow_velocity(self) -> TwistStamped:
        """Compute velocity to follow detected person."""
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = 'base_link'

        if self.person_position is None:
            return vel_cmd

        # Horizontal control: center person in frame
        # person_position.x = horizontal offset (positive = person is to the right)
        # person_position.y = vertical offset in image (positive = person is below center)
        error_x = -self.person_position[0]  # Negative to move toward person
        error_y = -self.person_position[1]

        # P control for position
        vel_x = self.p_gain * error_x
        vel_y = self.p_gain * error_y

        # Feedforward from person velocity (predictive following)
        if self.person_velocity is not None:
            vel_x += self.ff_gain * self.person_velocity[0]
            vel_y += self.ff_gain * self.person_velocity[1]

        # Clamp horizontal velocity
        h_speed = np.sqrt(vel_x**2 + vel_y**2)
        if h_speed > self.max_h_speed:
            scale = self.max_h_speed / h_speed
            vel_x *= scale
            vel_y *= scale

        # Vertical control: maintain target altitude
        vel_z = 0.0
        if self.current_altitude is not None:
            altitude_error = self.target_altitude - self.current_altitude
            vel_z = self.p_gain * altitude_error
            vel_z = np.clip(vel_z, -self.max_v_speed, self.max_v_speed)

        # Set velocity command
        vel_cmd.twist.linear.x = float(vel_x)
        vel_cmd.twist.linear.y = float(vel_y)
        vel_cmd.twist.linear.z = float(vel_z)

        return vel_cmd

    def compute_search_velocity(self) -> TwistStamped:
        """Compute velocity for searching (slow rotation or pattern)."""
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = 'base_link'

        # TODO: Implement search pattern
        # For now, just rotate slowly to scan for person
        vel_cmd.twist.angular.z = 0.3  # rad/s - slow yaw rotation

        # Maintain altitude during search
        if self.current_altitude is not None:
            altitude_error = self.target_altitude - self.current_altitude
            vel_cmd.twist.linear.z = float(np.clip(
                self.p_gain * altitude_error,
                -self.max_v_speed,
                self.max_v_speed
            ))

        return vel_cmd

    def compute_hover_velocity(self) -> TwistStamped:
        """Compute velocity to hover in place."""
        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = self.get_clock().now().to_msg()
        vel_cmd.header.frame_id = 'base_link'

        # Only altitude control while hovering
        if self.current_altitude is not None:
            altitude_error = self.target_altitude - self.current_altitude
            vel_cmd.twist.linear.z = float(np.clip(
                self.p_gain * altitude_error,
                -self.max_v_speed,
                self.max_v_speed
            ))

        return vel_cmd

    def activate(self):
        """Activate following mode."""
        self.mode = self.MODE_HOVER
        self.get_logger().info('Follower activated - entering HOVER mode')

    def deactivate(self):
        """Deactivate and return to idle."""
        self.mode = self.MODE_IDLE
        self.get_logger().info('Follower deactivated - entering IDLE mode')


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
