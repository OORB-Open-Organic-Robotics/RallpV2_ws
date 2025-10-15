import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
import time


class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.start_time = time.time()

    def broadcast(self):
        t = time.time() - self.start_time
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = 0.5 * math.cos(t * 0.2)
        transform.transform.translation.y = 0.5 * math.sin(t * 0.2)
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomPublisher()
    rate = node.create_rate(10)
    while rclpy.ok():
        node.broadcast()
        rclpy.spin_once(node, timeout_sec=0.1)


if __name__ == '__main__':
    main()
