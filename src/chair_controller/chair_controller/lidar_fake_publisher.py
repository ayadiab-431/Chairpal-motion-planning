import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class FakeLidarPublisher(Node):

    def __init__(self):
        super().__init__('lidar_fake_publisher')
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.5, self.publish_scan)
        self.get_logger().info("Fake LiDAR ready 👑")

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = math.pi / 18
        scan.range_min = 0.0
        scan.range_max = 10.0

        # بيانات وهمية، الرقم 0.3 يمثل عائق قريب
# بدلي الرقم 0.3 اللي يمثل العائق بحيث يكون بعيد شوية أو في النقاط الأمامية فقط
        scan.ranges = [1.0, 1.0, 1.0, 1.0, 0.3, 1.0, 1.0, 1.0, 1.0, 1.0]
        scan.intensities = []

        self.publisher.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
