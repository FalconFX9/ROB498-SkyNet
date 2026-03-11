#!/usr/bin/env python3
"""
Pose Logger Node for MARY Drone.

Buffers /mavros/vision_pose/pose samples at a configurable rate and saves
them as a .npy file on shutdown for offline 3D path analysis.

Saved array shape: (N, 8)  — columns: timestamp_s, x, y, z, qx, qy, qz, qw
"""

import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PoseLoggerNode(Node):
    """Buffers vision_pose samples and saves to .npy on shutdown."""

    def __init__(self):
        super().__init__('pose_logger_node')

        self.declare_parameter('log_dir', '')
        self.declare_parameter('log_rate', 5.0)  # Hz — how often to record a sample

        log_dir = self.get_parameter('log_dir').value
        log_rate = self.get_parameter('log_rate').value

        self._log_dir = log_dir
        self._buffer = []
        self._last_record_time = 0.0
        self._min_interval = 1.0 / max(log_rate, 0.1)

        if not log_dir:
            self.get_logger().warn('No log_dir specified — pose logging disabled')
            return

        os.makedirs(log_dir, exist_ok=True)
        self.get_logger().info(
            f'Pose logger ready — saving to {log_dir}/vision_pose.npy  '
            f'at {log_rate:.1f} Hz'
        )

        self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self._on_pose,
            10,
        )

    def _on_pose(self, msg: PoseStamped):
        if not self._log_dir:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self._last_record_time < self._min_interval:
            return
        self._last_record_time = t

        p = msg.pose.position
        q = msg.pose.orientation
        self._buffer.append([t, p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def destroy_node(self):
        if self._log_dir and self._buffer:
            data = np.array(self._buffer, dtype=np.float64)
            out_path = os.path.join(self._log_dir, 'vision_pose.npy')
            np.save(out_path, data)
            self.get_logger().info(
                f'Saved {len(self._buffer)} pose samples to {out_path}'
            )
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
