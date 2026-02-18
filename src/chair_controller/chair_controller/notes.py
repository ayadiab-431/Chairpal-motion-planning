import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        # Subscriber للـ LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Publisher لأوامر الحركة المعدلة
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # المسافة الآمنة
        self.safe_distance = 0.5  # متر

    def scan_callback(self, msg):
        twist = Twist()

        # فقط نركز على الجزء الأمامي من LiDAR بدل كل النقاط
        front_ranges = msg.ranges[3:7]  # تعديل: الجزء الأمامي فقط
        min_front = min(front_ranges)   # أقل مسافة أمام الكرسي

        if min_front < self.safe_distance:
            twist.linear.x = 0.0         # توقف عن الحركة الأمامية
            twist.angular.z = 0.5        # لف لتفادي العائق
            self.get_logger().info(f"Obstacle detected at {min_front:.2f} m, stopping!")
        else:
            twist.linear.x = 0.5         # حركة للأمام بسرعة مناسبة
            twist.angular.z = 0.0
            self.get_logger().info("Path clear, moving forward")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()