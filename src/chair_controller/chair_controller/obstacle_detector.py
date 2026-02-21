import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/teleop_cmd_vel', 10)

        # الإعدادات
        self.safe_distance = 1.5    # يوقف قبل الحيط بـ 1.5 متر (مسافة أمان)
        self.clear_distance = 2.0   # يتأكد إن الطريق فاضي بـ 2 متر قبل يروح قدام
        self.state = "WAIT"
        self.scan_count = 0
        self.backward_counter = 0   # عداد لإتمام مرحلة الرجوع
        self.clear_counter = 0      # عداد لتأكيد مسار فاضي

    def scan_callback(self, msg):
        twist = Twist()
        n = len(msg.ranges)
        if n == 0:
            return

        self.scan_count += 1

        # انتظار تحميل الحساس (أول ثانية)
        if self.scan_count < 15:
            self.cmd_pub.publish(twist)
            return
        elif self.scan_count == 15:
            self.state = "FORWARD"
            self.get_logger().info("Sensor ready! Starting FORWARD")

        # ===== المخروط الأمامي: indices 120-240 =====
        # مؤكد تجريبياً: linear.x = -0.4 يحرك الكرسي نحو index 180 (الأمام البصري)
        front_start = n // 3        # index 120
        front_end   = 2 * n // 3   # index 240
        front_ranges = [r for r in msg.ranges[front_start:front_end]
                        if 0.1 < r < 10.0]
        min_front = min(front_ranges) if front_ranges else 10.0

        # ========== STATE MACHINE ==========

        # --- 1) FORWARD ---
        if self.state == "FORWARD":
            if min_front < self.safe_distance:
                self.state = "BACKWARD"
                self.backward_counter = 0
                self.get_logger().info(f"Obstacle at {min_front:.2f}m! → BACKWARD")
            else:
                twist.linear.x = -0.3   # الأمام البصري (نحو index 180)
                twist.angular.z = 0.0
                self.get_logger().info(f"FORWARD | front={min_front:.2f}m")

        # --- 2) BACKWARD: يرجع 20 قراءة (≈2 ثانية) بغض النظر عن المسافة ---
        elif self.state == "BACKWARD":
            self.backward_counter += 1
            twist.linear.x = 0.2    # رجوع عكس الأمام البصري
            twist.angular.z = 0.0
            self.get_logger().info(f"BACKWARD ({self.backward_counter}/20)")
            if self.backward_counter >= 30:  # 30 قراءة ≈ 3 ثوانٍ
                self.state = "ROTATE"
                self.clear_counter = 0
                self.get_logger().info("→ ROTATE")

        # --- 3) ROTATE: يلف في مكانه حتى يرى طريق فاضي ---
        elif self.state == "ROTATE":
            twist.linear.x = 0.0
            twist.angular.z = 1.0   # دوران
            self.get_logger().info(f"ROTATE | front={min_front:.2f}m")
            if min_front > self.clear_distance:
                self.clear_counter += 1
            else:
                self.clear_counter = 0
            if self.clear_counter >= 5:   # 5 قراءات متتالية واضحة
                self.state = "FORWARD"
                self.get_logger().info("Path CLEAR → FORWARD")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()