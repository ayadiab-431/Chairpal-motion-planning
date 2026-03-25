#!/usr/bin/env python3
"""
TF Republisher: Strips a prefix from frame IDs and republishes TF.
Useful when Gazebo publishes TF with a model namespace (wheelchair/odom),
but ROS2 tools expect bare names (odom).
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

DEFAULT_STRIP_PREFIX = 'wheelchair/'

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        self.strip_prefix = self.declare_parameter('strip_prefix', DEFAULT_STRIP_PREFIX).value
        input_topic = self.declare_parameter('input_topic', '/tf_raw').value
        output_topic = self.declare_parameter('output_topic', '/tf').value
        self.sub = self.create_subscription(TFMessage, input_topic, self.tf_callback, 100)
        self.pub = self.create_publisher(TFMessage, output_topic, 100)
        self.get_logger().info(
            f'TF Republisher started — stripping "{self.strip_prefix}" from {input_topic} -> {output_topic}'
        )

    def tf_callback(self, msg: TFMessage):
        new_transforms = []
        for t in msg.transforms:
            parent = t.header.frame_id
            child = t.child_frame_id
            if self.strip_prefix:
                if parent.startswith(self.strip_prefix):
                    t.header.frame_id = parent.replace(self.strip_prefix, '', 1)
                if child.startswith(self.strip_prefix):
                    t.child_frame_id = child.replace(self.strip_prefix, '', 1)
            new_transforms.append(t)

        new_msg = TFMessage()
        new_msg.transforms = new_transforms
        self.pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
