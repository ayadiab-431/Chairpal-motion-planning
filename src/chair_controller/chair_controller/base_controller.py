import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')

        # Subscriber على /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)

        # Publisher داخلي للwheel_speeds
        self.wheel_publisher = self.create_publisher(
            Float32MultiArray,
            '/wheel_speeds',
            10)

        # Publisher للـ Simulation (TurtleSim)
        self.turtle_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        self.get_logger().info("Base Controller ready 👑")

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # حساب السرعات لكل عجلة
        left_speed  = linear - angular
        right_speed = linear + angular

        # نشر السرعات للحسابات الداخلية
        wheel_msg = Float32MultiArray()
        wheel_msg.data = [left_speed, right_speed]
        self.wheel_publisher.publish(wheel_msg)

        # نشر أمر الحركة للسلحفاة في Simulation
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        self.turtle_pub.publish(twist_msg)

        self.get_logger().info(
            f"Left: {left_speed:.2f} | Right: {right_speed:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
