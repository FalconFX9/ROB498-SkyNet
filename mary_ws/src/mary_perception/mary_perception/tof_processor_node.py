#!/usr/bin/env python3
"""
ToF (Time-of-Flight) Processor Node for MARY Drone
Processes TeraRanger Evo 60m sensor data for altitude hold and person distance.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped
import numpy as np
from collections import deque


class ToFProcessorNode(Node):
    """
    Processes TeraRanger Evo 60m ToF sensor data.

    Subscriptions:
        /mary/tof/range (sensor_msgs/Range): Raw ToF range measurement

    Publications:
        /mary/perception/altitude (std_msgs/Float32): Filtered altitude above ground/person
        /mary/perception/altitude_raw (std_msgs/Float32): Raw altitude measurement
        /mary/perception/altitude_velocity (std_msgs/Float32): Vertical velocity estimate
    """

    # TeraRanger Evo 60m specifications
    TOF_MIN_RANGE = 0.5    # meters
    TOF_MAX_RANGE = 60.0   # meters
    TOF_FOV = 2.0          # degrees (narrow beam)

    def __init__(self):
        super().__init__('tof_processor_node')

        # Parameters
        self.declare_parameter('filter_window_size', 5)
        self.declare_parameter('outlier_threshold', 0.5)  # meters
        self.declare_parameter('target_altitude', 2.0)     # meters above person
        self.declare_parameter('altitude_deadband', 0.1)   # meters
        self.declare_parameter('publish_rate', 20.0)       # Hz

        # Get parameters
        self.filter_window = self.get_parameter('filter_window_size').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value
        self.target_altitude = self.get_parameter('target_altitude').value

        # Filtering state
        self.range_history = deque(maxlen=self.filter_window)
        self.filtered_altitude = None
        self.last_altitude = None
        self.last_time = None

        # QoS profile
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.range_sub = self.create_subscription(
            Range,
            '/mary/tof/range',
            self.range_callback,
            sensor_qos
        )

        # Publishers
        self.altitude_pub = self.create_publisher(
            Float32,
            '/mary/perception/altitude',
            10
        )
        self.altitude_raw_pub = self.create_publisher(
            Float32,
            '/mary/perception/altitude_raw',
            10
        )
        self.altitude_vel_pub = self.create_publisher(
            Float32,
            '/mary/perception/altitude_velocity',
            10
        )

        self.get_logger().info(f'ToF Processor Node initialized')
        self.get_logger().info(f'  Target altitude: {self.target_altitude}m')
        self.get_logger().info(f'  Filter window: {self.filter_window} samples')

    def range_callback(self, msg: Range):
        """Process incoming ToF range measurement."""
        current_time = self.get_clock().now()

        # Validate range
        if msg.range < self.TOF_MIN_RANGE or msg.range > self.TOF_MAX_RANGE:
            self.get_logger().warn(f'ToF range out of bounds: {msg.range:.2f}m')
            return

        # Publish raw altitude
        raw_msg = Float32()
        raw_msg.data = msg.range
        self.altitude_raw_pub.publish(raw_msg)

        # Add to history for filtering
        self.range_history.append(msg.range)

        # Apply median filter for outlier rejection
        if len(self.range_history) >= 3:
            self.filtered_altitude = self.median_filter()
        else:
            self.filtered_altitude = msg.range

        # Publish filtered altitude
        alt_msg = Float32()
        alt_msg.data = self.filtered_altitude
        self.altitude_pub.publish(alt_msg)

        # Calculate and publish vertical velocity
        if self.last_altitude is not None and self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9
            if dt > 0:
                velocity = (self.filtered_altitude - self.last_altitude) / dt
                vel_msg = Float32()
                vel_msg.data = velocity
                self.altitude_vel_pub.publish(vel_msg)

        # Update state
        self.last_altitude = self.filtered_altitude
        self.last_time = current_time

    def median_filter(self):
        """Apply median filter to range history."""
        return float(np.median(list(self.range_history)))

    def is_valid_measurement(self, value):
        """Check if measurement is within expected bounds."""
        if self.filtered_altitude is None:
            return True
        return abs(value - self.filtered_altitude) < self.outlier_threshold


def main(args=None):
    rclpy.init(args=args)
    node = ToFProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
