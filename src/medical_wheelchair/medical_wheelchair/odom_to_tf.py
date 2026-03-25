#!/usr/bin/env python3
"""
Odom -> TF broadcaster.
Subscribes to nav_msgs/Odometry and publishes the equivalent transform on /tf.
Optionally strips a namespace prefix from frame IDs.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

DEFAULT_STRIP_PREFIX = 'wheelchair/'


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.odom_frame = self.declare_parameter('odom_frame', '').value
        self.base_frame = self.declare_parameter('base_frame', '').value
        self.strip_prefix = self.declare_parameter('strip_prefix', DEFAULT_STRIP_PREFIX).value
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)

        self.get_logger().info(
            f'OdomToTF started — listening on {self.odom_topic}, '
            f'strip_prefix="{self.strip_prefix}"'
        )

    def odom_callback(self, msg: Odometry):
        parent = self.odom_frame or msg.header.frame_id
        child = self.base_frame or msg.child_frame_id

        if self.strip_prefix:
            if parent.startswith(self.strip_prefix):
                parent = parent.replace(self.strip_prefix, '', 1)
            if child.startswith(self.strip_prefix):
                child = child.replace(self.strip_prefix, '', 1)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
