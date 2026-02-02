#!/usr/bin/env python3
"""
Altitude Controller Node for MARY Drone
PID controller for maintaining altitude above person using ToF sensor.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
from collections import deque


class AltitudeControllerNode(Node):
    """
    PID altitude controller using ToF sensor feedback.

    Subscriptions:
        /mary/perception/altitude (std_msgs/Float32): Current altitude
        /mary/control/altitude_setpoint (std_msgs/Float32): Target altitude

    Publications:
        /mary/control/altitude_command (std_msgs/Float32): Vertical velocity command
        /mary/control/altitude_error (std_msgs/Float32): Current altitude error
    """

    def __init__(self):
        super().__init__('altitude_controller_node')

        # PID Parameters
        self.declare_parameter('kp', 1.5)           # Proportional gain
        self.declare_parameter('ki', 0.1)           # Integral gain
        self.declare_parameter('kd', 0.5)           # Derivative gain
        self.declare_parameter('max_integral', 2.0) # Anti-windup limit
        self.declare_parameter('max_output', 1.5)   # Max vertical velocity (m/s)
        self.declare_parameter('deadband', 0.05)    # Error deadband (m)
        self.declare_parameter('control_rate', 50.0)  # Hz

        # Safety parameters
        self.declare_parameter('min_altitude', 1.0)  # Minimum safe altitude (m)
        self.declare_parameter('max_altitude', 5.0)  # Maximum altitude (m)
        self.declare_parameter('default_setpoint', 2.5)  # Default target altitude

        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_integral = self.get_parameter('max_integral').value
        self.max_output = self.get_parameter('max_output').value
        self.deadband = self.get_parameter('deadband').value
        self.min_altitude = self.get_parameter('min_altitude').value
        self.max_altitude = self.get_parameter('max_altitude').value
        self.setpoint = self.get_parameter('default_setpoint').value

        # PID state
        self.current_altitude = None
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

        # Derivative filter (moving average)
        self.derivative_history = deque(maxlen=5)

        # Subscribers
        self.altitude_sub = self.create_subscription(
            Float32,
            '/mary/perception/altitude',
            self.altitude_callback,
            10
        )
        self.setpoint_sub = self.create_subscription(
            Float32,
            '/mary/control/altitude_setpoint',
            self.setpoint_callback,
            10
        )

        # Publishers
        self.command_pub = self.create_publisher(
            Float32,
            '/mary/control/altitude_command',
            10
        )
        self.error_pub = self.create_publisher(
            Float32,
            '/mary/control/altitude_error',
            10
        )

        # Control timer
        control_rate = self.get_parameter('control_rate').value
        self.create_timer(1.0 / control_rate, self.control_loop)

        self.get_logger().info('Altitude Controller Node initialized')
        self.get_logger().info(f'  PID gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
        self.get_logger().info(f'  Altitude limits: [{self.min_altitude}, {self.max_altitude}]m')

    def altitude_callback(self, msg: Float32):
        """Update current altitude measurement."""
        self.current_altitude = msg.data

    def setpoint_callback(self, msg: Float32):
        """Update altitude setpoint with safety limits."""
        new_setpoint = np.clip(msg.data, self.min_altitude, self.max_altitude)
        if new_setpoint != msg.data:
            self.get_logger().warn(f'Setpoint clipped from {msg.data:.2f} to {new_setpoint:.2f}m')
        self.setpoint = new_setpoint

    def control_loop(self):
        """PID control loop."""
        if self.current_altitude is None:
            return

        current_time = self.get_clock().now()

        # Calculate error
        error = self.setpoint - self.current_altitude

        # Publish error for debugging
        error_msg = Float32()
        error_msg.data = error
        self.error_pub.publish(error_msg)

        # Apply deadband
        if abs(error) < self.deadband:
            error = 0.0

        # Calculate dt
        if self.last_time is not None:
            dt = (current_time - self.last_time).nanoseconds * 1e-9
        else:
            dt = 0.02  # Default to 50Hz

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.max_integral, self.max_integral)
        i_term = self.ki * self.integral

        # Derivative term with filtering
        if dt > 0:
            derivative = (error - self.last_error) / dt
            self.derivative_history.append(derivative)
            filtered_derivative = np.mean(self.derivative_history)
            d_term = self.kd * filtered_derivative
        else:
            d_term = 0.0

        # Calculate output
        output = p_term + i_term + d_term

        # Clamp output
        output = np.clip(output, -self.max_output, self.max_output)

        # Safety check: prevent going below minimum altitude
        if self.current_altitude < self.min_altitude and output < 0:
            output = 0.0
            self.get_logger().warn('Minimum altitude reached - blocking descent')

        # Publish command
        cmd_msg = Float32()
        cmd_msg.data = float(output)
        self.command_pub.publish(cmd_msg)

        # Update state
        self.last_error = error
        self.last_time = current_time

    def reset(self):
        """Reset PID state."""
        self.integral = 0.0
        self.last_error = 0.0
        self.derivative_history.clear()
        self.get_logger().info('PID controller reset')


def main(args=None):
    rclpy.init(args=args)
    node = AltitudeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
