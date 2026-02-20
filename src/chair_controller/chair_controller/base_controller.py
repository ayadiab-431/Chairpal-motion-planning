import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')

        # Subscriber على /teleop_cmd_vel بدلاً من /cmd_vel عشان مانعملش تداخل مع الخرج النهائي
        self.subscription = self.create_subscription(
            Twist,
            '/teleop_cmd_vel',
            self.cmd_callback,
            10)

        # Publisher داخلي للwheel_speeds
        self.wheel_publisher = self.create_publisher(
            Float32MultiArray,
            '/wheel_speeds',
            10)

        self.get_logger().info("Base Controller ready 👑")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # حساب السرعات الأساسية لليمين واليسار
        left_base_speed = linear - angular
        right_base_speed = linear + angular

        # توزيع السرعات على الـ 6 مواتير (3 يمين، 3 شمال)
        # تقدر تعدل القيم دي لو المواتير الأمامية أو الخلفية محتاجة نسبة سرعة مختلفة
        front_left_speed = left_base_speed
        middle_left_speed = left_base_speed
        rear_left_speed = left_base_speed

        front_right_speed = right_base_speed
        middle_right_speed = right_base_speed
        rear_right_speed = right_base_speed

        # نشر الـ 6 سرعات
        # الترتيب: [أمام يسار, وسط يسار, خلف يسار, أمام يمين, وسط يمين, خلف يمين]
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [
            front_left_speed, middle_left_speed, rear_left_speed,
            front_right_speed, middle_right_speed, rear_right_speed
        ]
        self.wheel_publisher.publish(wheel_msg)

        self.get_logger().info(
            f"Motors L: [{front_left_speed:.2f}, {middle_left_speed:.2f}, {rear_left_speed:.2f}] | "
            f"Motors R: [{front_right_speed:.2f}, {middle_right_speed:.2f}, {rear_right_speed:.2f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
