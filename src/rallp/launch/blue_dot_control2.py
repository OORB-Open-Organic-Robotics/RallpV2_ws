#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bluedot import BlueDot
from geometry_msgs.msg import Twist

class LeftJoystickControl(Node):
    def __init__(self):
        super().__init__('left_joystick_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 100)
        self.bd = BlueDot(cols=3, rows=1)
        self.bd[1,0].visible = False
        self.left_joystick = self.bd[0,0]
        
        self.left_joystick.when_moved = self.on_joystick_move
        self.bd.when_released = self.on_release

    def on_joystick_move(self, pos):
        twist = Twist()
        twist.linear.x = pos.y * 2.0 
        twist.angular.z = pos.x * (-2.0)
        self.publisher.publish(twist)
        self.get_logger().info(f"Control: linear={twist.linear.x:.1f}, angular={twist.angular.z:.1f}")

    def on_release(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info("Robot stopped")

def main(args=None):
    rclpy.init(args=args)
    node = LeftJoystickControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()