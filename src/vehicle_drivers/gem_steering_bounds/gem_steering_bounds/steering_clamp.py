#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pacmod2_msgs.msg import PositionWithSpeed


class SteeringClampNode(Node):
    """
    Relay node that clamps steering angle commands before they reach the PACMod driver.

    Subscribes to /pacmod/steering_cmd_raw (remapped output from game_control or
    autonomous stack) and republishes to /pacmod/steering_cmd with angular_position
    clamped to [-max_steering_angle, +max_steering_angle].

    The Polaris GEM e4 physical limit is 8.5 rad (hardcoded in pacmod2_game_control).
    Default clamp of 7.5 rad leaves ~1 rad of buffer from the mechanical stops.
    """

    def __init__(self):
        super().__init__('steering_clamp')

        self.declare_parameter('max_steering_angle', 7.5)
        self.max_angle = self.get_parameter('max_steering_angle').value

        self.get_logger().info(
            f'Steering clamp active — max angle: ±{self.max_angle:.2f} rad '
            f'(physical limit: ±8.5 rad)'
        )

        self.pub = self.create_publisher(PositionWithSpeed, '/pacmod/steering_cmd', 10)
        self.sub = self.create_subscription(
            PositionWithSpeed,
            '/pacmod/steering_cmd_raw',
            self.on_cmd,
            10,
        )

    def on_cmd(self, msg: PositionWithSpeed):
        clamped = max(-self.max_angle, min(self.max_angle, msg.angular_position))
        if clamped != msg.angular_position:
            self.get_logger().warn(
                f'Steering clamped: {msg.angular_position:.3f} → {clamped:.3f} rad',
                throttle_duration_sec=1.0,
            )
        out = PositionWithSpeed()
        out.angular_position = clamped
        out.angular_velocity_limit = msg.angular_velocity_limit
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringClampNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
