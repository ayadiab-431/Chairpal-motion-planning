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

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ========================== إعدادات ==========================
        self.safe_distance  = 1.2    # يبدأ التفادي عند 1.2 متر (قبل ما يخبط بكثير)
        self.clear_distance = 1.0    # الطريق فاضي لو > 1.0 متر
        self.forward_speed  = 0.4    # الأمام (+x في /cmd_vel)
        self.backward_speed = -0.3   # الخلف (-x في /cmd_vel)
        self.turn_speed     = 3.5    # سرعة دوران قوية (مؤكد تجريبياً)
        self.arc_turn_speed  = 2.0    # سرعة دوران أثناء الرجوع (قوس)
        self.arc_ticks       = 30    # عدد القراءات للرجوع مع دوران (~0.5 ثانية)

        # 90 درجة بتحتاج ~6 ثواني عند z=3.5 → 60 قراءة
        self.backward_ticks = 8   # تقريباً 0.8 ثانية للخلف (بدون دوران)
        self.rotate_ticks   = 60  # تقريباً 6 ثواني دوران (~90-120 درجة)

        self.state = "WAIT"
        self.scan_count = 0
        self.counter = 0
        self.turn_sign = 1.0

    def scan_callback(self, msg):
        twist = Twist()
        n = len(msg.ranges)
        if n == 0:
            return

        self.scan_count += 1

        if self.scan_count < 20:
            self.cmd_pub.publish(twist)
            return
        elif self.scan_count == 20:
            self.state = "FORWARD"
            self.get_logger().info("Sensor ready! → FORWARD")

        # ===== مخروط الأمام (~80 درجة) =====
        half    = n // 2
        cone    = n // 9
        front_s = half - cone
        front_e = half + cone

        front_ranges = [r for r in msg.ranges[front_s:front_e]
                        if 0.1 < r < 10.0]
        min_front = min(front_ranges) if front_ranges else 10.0

        # =================== STATE MACHINE ===================

        if self.state == "FORWARD":
            if min_front < self.safe_distance:
                # اختر الاتجاه الأوسع
                left_r  = [r for r in msg.ranges[front_e:front_e+60]
                           if 0.1 < r < 10.0]
                right_r = [r for r in msg.ranges[max(0, front_s-60):front_s]
                           if 0.1 < r < 10.0]
                l_avg = sum(left_r)  / len(left_r)  if left_r  else 0.5
                r_avg = sum(right_r) / len(right_r) if right_r else 0.5
                self.turn_sign = 1.0 if l_avg > r_avg else -1.0

                self.state = "BACKWARD"
                self.counter = 0
                self.get_logger().info(
                    f"Obstacle {min_front:.2f}m! "
                    f"→ BACKWARD then turn {'LEFT' if self.turn_sign > 0 else 'RIGHT'}")
            else:
                twist.linear.x  = self.forward_speed
                twist.angular.z = 0.0
                self.get_logger().info(f"FORWARD | {min_front:.2f}m")

        elif self.state == "BACKWARD":
            self.counter += 1
            # Move backward while turning (arc)
            twist.linear.x  = self.backward_speed
            twist.angular.z = self.turn_sign * self.arc_turn_speed
            self.get_logger().info(f"BACKWARD ARC ({self.counter}/{self.arc_ticks})")
            if self.counter >= self.arc_ticks:
                # After arc, switch to ROTATE to reorient further if needed
                self.state = "ROTATE"
                self.counter = 0
                self.get_logger().info("→ ROTATE after arc")

        elif self.state == "ROTATE":
            self.counter += 1
            twist.linear.x  = 0.0
            twist.angular.z = self.turn_sign * self.turn_speed
            self.get_logger().info(
                f"ROTATE ({self.counter}/{self.rotate_ticks}) "
                f"front={min_front:.2f}m z={twist.angular.z:.1f}")
            if self.counter >= self.rotate_ticks:
                self.state = "FORWARD"
                self.counter = 0
                self.get_logger().info("→ FORWARD")

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()