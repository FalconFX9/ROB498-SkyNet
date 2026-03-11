#!/usr/bin/env python3
"""
Pose Logger Node for MARY Drone.

Subscribes to /mavros/vision_pose/pose and writes each pose to a CSV file
for offline 3D path analysis.  Activated via the record_flight.sh wrapper.

CSV columns: timestamp_s, x, y, z, qx, qy, qz, qw
"""

import csv
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseLoggerNode(Node):
    """Logs /mavros/vision_pose/pose to a timestamped CSV file."""

    def __init__(self):
        super().__init__('pose_logger_node')

        self.declare_parameter('log_dir', '')

        log_dir = self.get_parameter('log_dir').value

        self._csv_writer = None
        self._csv_file = None

        if not log_dir:
            self.get_logger().warn('No log_dir specified — pose logging disabled')
            return

        os.makedirs(log_dir, exist_ok=True)
        filepath = os.path.join(log_dir, 'vision_pose.csv')

        self._csv_file = open(filepath, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['timestamp_s', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        self.get_logger().info(f'Logging vision_pose to {filepath}')

        self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self._on_pose,
            10,
        )

    def _on_pose(self, msg: PoseStamped):
        if self._csv_writer is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        q = msg.pose.orientation
        self._csv_writer.writerow([
            f'{t:.6f}', f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}',
        ])

    def destroy_node(self):
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self.get_logger().info('Pose log file closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
