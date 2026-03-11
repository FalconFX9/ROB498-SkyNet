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
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
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
        self.declare_parameter('publish_to_mavros', True)  # Set False when another node relays
        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('drone_frame_id', 'base_link')

        # T265 → ENU frame rotation (180° about Z axis)
        # T265 mounted on bottom of drone, facing DOWN, lenses BACKWARD.
        # Empirically verified mapping to ENU for MAVROS vision_pose:
        #   ENU +X (East  / right)   = -T265 X
        #   ENU +Y (North / forward) = -T265 Y
        #   ENU +Z (Up)              = +T265 Z
        self.declare_parameter('camera_position_x', 0.0)  # Camera offset from drone center
        self.declare_parameter('camera_position_y', 0.0)
        self.declare_parameter('camera_position_z', 0.0)
        self.declare_parameter('log_euler', False)       # Log RPY at ~2 Hz

        # Get parameters
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_to_mavros = self.get_parameter('publish_to_mavros').value
        self.world_frame = self.get_parameter('world_frame_id').value
        self.drone_frame = self.get_parameter('drone_frame_id').value

        # Precompute T265 → ENU frame rotation (used every callback)
        self._R_t265_to_enu = np.array([
            [1,  0, 0],
            [ 0, 1, 0],
            [ 0,  0, 1],
        ])
        # Equivalent quaternion [x, y, z, w] for the same rotation
        self._q_t265_to_enu = tf_transformations.quaternion_from_matrix(
            np.vstack([
                np.hstack([self._R_t265_to_enu, np.zeros((3, 1))]),
                [0, 0, 0, 1]
            ])
        )

        self.log_euler = self.get_parameter('log_euler').value
        self._euler_log_counter = 0

        # Debug RPY publishers (created only when log_euler is enabled)
        if self.log_euler:
            self._pub_t265_raw_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/t265_raw_rpy', 10)
            self._pub_t265_enu_rpy = self.create_publisher(
                Vector3Stamped, '/mary/debug/t265_enu_rpy', 10)

        # State
        self.current_pose = None
        self.initial_pose = None  # For zeroing position at start
        self.pose_initialized = False

        # Orientation zeroing: collect samples then compute inverse
        self._orient_samples = []          # list of [x, y, z, w] quaternions
        self._orient_sample_count = 100    # ~0.5 s at 200 Hz
        self._q_orient_zero_inv = None     # inverse of initial ENU orientation

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

        # Validate quaternion — T265 publishes zeros during SLAM errors
        quat_norm = np.linalg.norm(orientation)
        if quat_norm < 1e-6:
            return  # Skip invalid pose silently
        orientation = orientation / quat_norm  # Normalize

        # Collect initial samples to zero position and orientation
        if not self.pose_initialized:
            if self.initial_pose is None:
                self.initial_pose = {
                    'position': position.copy(),
                    'orientation': orientation.copy()
                }
            self._orient_samples.append(orientation.copy())
            if len(self._orient_samples) < self._orient_sample_count:
                return  # Keep collecting
            # Average the collected quaternions and compute ENU inverse
            avg_q = self._average_quaternions(self._orient_samples)
            # Transform the averaged raw orientation to ENU
            avg_enu = tf_transformations.quaternion_multiply(
                self._q_t265_to_enu, avg_q)
            # Negate yaw (same as in transform_t265_to_drone)
            r, p, y = tf_transformations.euler_from_quaternion(avg_enu)
            avg_enu = tf_transformations.quaternion_from_euler(r, p, -y)
            # Store the conjugate (inverse) to zero future orientations
            self._q_orient_zero_inv = tf_transformations.quaternion_conjugate(avg_enu)
            self.pose_initialized = True
            rpy_deg = np.degrees([r, p, -y])
            self.get_logger().info(
                f'Initial pose captured ({len(self._orient_samples)} samples) — '
                f'zeroing position and orientation '
                f'(initial ENU RPY: {rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f})'
            )
            self._orient_samples = []  # Free memory

        # Transform from T265 frame to ENU
        transformed_pose = self.transform_t265_to_drone(position, orientation)
        if transformed_pose is None:
            return

        # Store current pose
        self.current_pose = {
            'position': transformed_pose['position'],
            'orientation': transformed_pose['orientation'],
            'header': msg.header
        }

        # Publish debug RPY (~10 Hz from ~200 Hz callback)
        if self.log_euler:
            self._euler_log_counter += 1
            if self._euler_log_counter % 20 == 0:
                stamp = self.get_clock().now().to_msg()
                raw_rpy = np.degrees(tf_transformations.euler_from_quaternion(orientation))
                out_rpy = np.degrees(tf_transformations.euler_from_quaternion(
                    transformed_pose['orientation']))

                msg_raw = Vector3Stamped()
                msg_raw.header.stamp = stamp
                msg_raw.vector.x, msg_raw.vector.y, msg_raw.vector.z = \
                    float(raw_rpy[0]), float(raw_rpy[1]), float(raw_rpy[2])
                self._pub_t265_raw_rpy.publish(msg_raw)

                msg_enu = Vector3Stamped()
                msg_enu.header.stamp = stamp
                msg_enu.vector.x, msg_enu.vector.y, msg_enu.vector.z = \
                    float(out_rpy[0]), float(out_rpy[1]), float(out_rpy[2])
                self._pub_t265_enu_rpy.publish(msg_enu)

    @staticmethod
    def _average_quaternions(quats):
        """Average quaternions using eigenvector method (Markley et al.)."""
        Q = np.array(quats)  # Nx4, [x, y, z, w]
        # Ensure all quaternions are in the same hemisphere
        for i in range(1, len(Q)):
            if np.dot(Q[i], Q[0]) < 0:
                Q[i] = -Q[i]
        M = Q.T @ Q  # 4x4
        eigenvalues, eigenvectors = np.linalg.eigh(M)
        avg = eigenvectors[:, -1]  # Largest eigenvalue
        return avg / np.linalg.norm(avg)

    def transform_t265_to_drone(self, position, orientation):
        """Transform T265 pose to ENU frame for MAVROS."""
        # Transform position
        if self.initial_pose is not None:
            relative_pos = position - self.initial_pose['position']
        else:
            relative_pos = position

        transformed_position = self._R_t265_to_enu @ relative_pos

        # Transform orientation via quaternion multiplication
        transformed_orientation = tf_transformations.quaternion_multiply(
            self._q_t265_to_enu, orientation
        )

        # Negate yaw (flip yaw direction while preserving roll/pitch)
        r, p, y = tf_transformations.euler_from_quaternion(transformed_orientation)
        transformed_orientation = tf_transformations.quaternion_from_euler(r, p, -y)

        # Zero orientation relative to initial pose
        if self._q_orient_zero_inv is not None:
            transformed_orientation = tf_transformations.quaternion_multiply(
                self._q_orient_zero_inv, transformed_orientation
            )

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

        # Publish to MAVROS (can be disabled when another node handles relay)
        if self.publish_to_mavros:
            self.mavros_pose_pub.publish(pose_msg)

        # Publish to MARY localization topic (always)
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
