#!/usr/bin/env python3
"""
TF Republisher: Strips 'wheelchair/' prefix from frame IDs.
Gazebo publishes TF with model namespace (wheelchair/odom, wheelchair/base_footprint),
but ROS2 and SLAM expect bare names (odom, base_footprint).
This node bridges that gap.
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

STRIP_PREFIX = 'wheelchair/'

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        self.sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 100)
        self.pub = self.create_publisher(TFMessage, '/tf', 100)
        self.get_logger().info('TF Republisher started — stripping "wheelchair/" prefix')

    def tf_callback(self, msg: TFMessage):
        modified = False
        new_transforms = []
        for t in msg.transforms:
            parent = t.header.frame_id
            child = t.child_frame_id
            if parent.startswith(STRIP_PREFIX) or child.startswith(STRIP_PREFIX):
                t.header.frame_id = parent.replace(STRIP_PREFIX, '', 1)
                t.child_frame_id = child.replace(STRIP_PREFIX, '', 1)
                modified = True
            new_transforms.append(t)

        if modified:
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
