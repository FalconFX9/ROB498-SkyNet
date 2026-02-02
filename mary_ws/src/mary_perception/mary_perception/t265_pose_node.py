#!/usr/bin/env python3
"""
T265 Pose Node for MARY Drone
Processes Intel RealSense T265 tracking camera data for drone localization.
Publishes pose to MAVROS for visual-inertial odometry.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_system_default
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import tf_transformations


class T265PoseNode(Node):
    """
    Processes T265 pose data and publishes to MAVROS.

    Subscriptions:
        /camera/pose/sample (nav_msgs/Odometry): T265 pose output

    Publications:
        /mavros/vision_pose/pose (geometry_msgs/PoseStamped): Vision pose for PX4
        /mary/localization/pose (geometry_msgs/PoseStamped): Drone pose in world frame
    """

    def __init__(self):
        super().__init__('t265_pose_node')

        # Parameters
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('drone_frame_id', 'base_link')

        # T265 to drone frame transformation
        # T265 coordinate system: X-right, Y-down, Z-forward
        # Drone NED: X-forward, Y-right, Z-down
        self.declare_parameter('camera_position_x', 0.0)  # Camera offset from drone center
        self.declare_parameter('camera_position_y', 0.0)
        self.declare_parameter('camera_position_z', 0.0)

        # Get parameters
        self.publish_tf = self.get_parameter('publish_tf').value
        self.world_frame = self.get_parameter('world_frame_id').value
        self.drone_frame = self.get_parameter('drone_frame_id').value

        # State
        self.current_pose = None
        self.initial_pose = None  # For zeroing position at start
        self.pose_initialized = False

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.pose_sub = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.pose_callback,
            qos_profile_system_default
        )

        # Publishers
        self.mavros_pose_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            qos_profile_system_default
        )
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mary/localization/pose',
            10
        )

        # Timer for publishing at fixed rate
        publish_rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / publish_rate, self.publish_pose)

        self.get_logger().info('T265 Pose Node initialized')
        self.get_logger().info(f'  Publishing to MAVROS at {publish_rate} Hz')

    def pose_callback(self, msg: Odometry):
        """Process incoming T265 pose."""
        # Extract position
        position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Extract orientation as quaternion [x, y, z, w]
        orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])

        # Store initial pose for zeroing
        if not self.pose_initialized:
            self.initial_pose = {
                'position': position.copy(),
                'orientation': orientation.copy()
            }
            self.pose_initialized = True
            self.get_logger().info('Initial pose captured - zeroing position')

        # Transform from T265 frame to drone frame
        transformed_pose = self.transform_t265_to_drone(position, orientation)

        # Store current pose
        self.current_pose = {
            'position': transformed_pose['position'],
            'orientation': transformed_pose['orientation'],
            'header': msg.header
        }

    def transform_t265_to_drone(self, position, orientation):
        """
        Transform T265 pose to drone body frame.

        T265 uses: X-right, Y-down, Z-forward
        Drone NED: X-forward, Y-right, Z-down
        """
        # Rotation matrix from T265 to drone frame
        # This maps T265 axes to drone axes
        R_t265_to_drone = np.array([
            [0, 0, 1],   # Drone X = T265 Z (forward)
            [1, 0, 0],   # Drone Y = T265 X (right)
            [0, 1, 0]    # Drone Z = T265 Y (down)
        ])

        # Transform position
        if self.initial_pose is not None:
            # Zero to initial position
            relative_pos = position - self.initial_pose['position']
        else:
            relative_pos = position

        transformed_position = R_t265_to_drone @ relative_pos

        # Transform orientation
        # Convert quaternion to rotation matrix, apply transform, convert back
        R_original = tf_transformations.quaternion_matrix(orientation)[:3, :3]
        R_transformed = R_t265_to_drone @ R_original

        # Pad to 4x4 for quaternion conversion
        R_4x4 = np.eye(4)
        R_4x4[:3, :3] = R_transformed
        transformed_orientation = tf_transformations.quaternion_from_matrix(R_4x4)

        return {
            'position': transformed_position,
            'orientation': transformed_orientation
        }

    def publish_pose(self):
        """Publish current pose at fixed rate."""
        if self.current_pose is None:
            return

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.world_frame

        pose_msg.pose.position.x = float(self.current_pose['position'][0])
        pose_msg.pose.position.y = float(self.current_pose['position'][1])
        pose_msg.pose.position.z = float(self.current_pose['position'][2])

        pose_msg.pose.orientation.x = float(self.current_pose['orientation'][0])
        pose_msg.pose.orientation.y = float(self.current_pose['orientation'][1])
        pose_msg.pose.orientation.z = float(self.current_pose['orientation'][2])
        pose_msg.pose.orientation.w = float(self.current_pose['orientation'][3])

        # Publish to MAVROS
        self.mavros_pose_pub.publish(pose_msg)

        # Publish to MARY localization topic
        self.pose_pub.publish(pose_msg)

        # Publish TF if enabled
        if self.publish_tf:
            self.publish_transform(pose_msg)

    def publish_transform(self, pose_msg: PoseStamped):
        """Publish TF transform."""
        t = TransformStamped()
        t.header = pose_msg.header
        t.child_frame_id = self.drone_frame

        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z
        t.transform.rotation = pose_msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = T265PoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
