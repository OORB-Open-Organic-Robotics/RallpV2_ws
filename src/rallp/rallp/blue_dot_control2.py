#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from bluedot import BlueDot


class BlueDotTeleop(Node):
    """Publish geometry_msgs/Twist commands in response to BlueDot input."""

    def __init__(self) -> None:
        super().__init__('blue_dot_control')
        self.declare_parameter('linear_scale', 1.5)
        self.declare_parameter('angular_scale', 1.5)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bd = BlueDot()
        self.bd.when_client_connects = self.on_connect
        self.bd.when_client_disconnects = self.on_disconnect
        self.bd.when_pressed = self.on_interaction
        self.bd.when_moved = self.on_interaction
        self.bd.when_released = self.on_release

        self.get_logger().info(
            'BlueDot teleop ready. Open the app, connect, then drag to drive.'
        )

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher.publish(twist)
        self.get_logger().info(
            f'cmd_vel -> linear: {linear_x:.2f} m/s, angular: {angular_z:.2f} rad/s',
            throttle_duration_sec=0.5,
        )

    def on_interaction(self, pos) -> None:
        if pos is None:
            return
        linear = pos.y * self.linear_scale
        angular = -pos.x * self.angular_scale  # Positive X on screen = turn left
        self.publish_twist(linear, angular)

    def on_release(self) -> None:
        self.publish_twist(0.0, 0.0)
        self.get_logger().info('BlueDot released – stopping robot.')

    def on_connect(self):
        self.get_logger().info('BlueDot client connected.')

    def on_disconnect(self):
        self.get_logger().info('BlueDot client disconnected.')
        self.publish_twist(0.0, 0.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueDotTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
