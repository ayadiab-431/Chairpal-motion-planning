#!/usr/bin/env python3
"""
LaserScan republisher to normalize frame_id for SLAM.
Some GZ sensors publish frames like 'wheelchair/base_footprint/gpu_lidar',
which don't exist in the TF tree. This node rewrites frame_id to a usable one.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

DEFAULT_STRIP_PREFIX = 'wheelchair/'


class ScanRepublisher(Node):
    def __init__(self):
        super().__init__('scan_republisher')
        self.strip_prefix = self.declare_parameter('strip_prefix', DEFAULT_STRIP_PREFIX).value
        self.input_topic = self.declare_parameter('input_topic', '/scan').value
        self.output_topic = self.declare_parameter('output_topic', '/scan_fixed').value
        self.frame_id = self.declare_parameter('frame_id', 'lidar_link').value

        self.sub = self.create_subscription(LaserScan, self.input_topic, self.cb, 50)
        self.pub = self.create_publisher(LaserScan, self.output_topic, 50)
        self.get_logger().info(
            f'ScanRepublisher started — {self.input_topic} -> {self.output_topic}, frame_id="{self.frame_id}"'
        )

    def cb(self, msg: LaserScan):
        if self.strip_prefix and msg.header.frame_id.startswith(self.strip_prefix):
            msg.header.frame_id = msg.header.frame_id.replace(self.strip_prefix, '', 1)
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
