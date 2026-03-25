import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        # Subscriber للـ LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # Subscriber لأوامر الحركة اليدوية (Teleop)
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_teleop',
            self.cmd_callback,
            10)

        # Publisher لأوامر الحركة النهائية (المفلترة)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_teleop_cmd = Twist()
        self.obstacle_detected = False
        self.get_logger().info("Safety Monitor initialized. Waiting for /cmd_vel_teleop commands... 🛡️")

        # المسافة الآمنة
        self.safe_distance = 0.5  # متر

    def scan_callback(self, msg):
        # فقط نركز على الجزء الأمامي من LiDAR
        front_ranges = msg.ranges[3:7]  
        min_front = min(front_ranges)   

        if min_front < self.safe_distance:
            self.obstacle_detected = True
            # إذا وجد عائق، نقوم بإيقاف الحركة الأمامية فوراً
            safe_twist = Twist()
            safe_twist.linear.x = 0.0
            safe_twist.angular.z = 0.3 # التفاف بطيء لتنبيه السائق
            self.cmd_pub.publish(safe_twist)
            self.get_logger().warn(f"OBSTACLE DETECTED at {min_front:.2f} m! Overriding command for safety.")
        else:
            self.obstacle_detected = False

    def cmd_callback(self, msg):
        # هذه الدالة تستقبل أوامر السائق وتمررها فقط إذا لم يكن هناك عائق
        if not self.obstacle_detected:
            self.cmd_pub.publish(msg)
        else:
            # إذا كان هناك عائق، نمنع فقط الحركة للأمام ونسمح بالدوران/الخلف
            if msg.linear.x > 0:
                self.get_logger().info("Forward motion blocked by safety monitor.")
            else:
                self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
