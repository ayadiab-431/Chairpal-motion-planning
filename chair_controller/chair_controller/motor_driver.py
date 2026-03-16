import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')

        # Subscriber للـ wheel speeds
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_speeds',
            self.speed_callback,
            10)

        # Publisher للـ Simulation
        self.turtle_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10)

        self.get_logger().info("Motor Driver ready 👑")

    def speed_callback(self, msg):
        left_speed = msg.data[0]
        right_speed = msg.data[1]

        # تحويل السرعات لأمر Twist للسلحفاة
        twist_msg = Twist()
        twist_msg.linear.x = (left_speed + right_speed) / 2.0
        twist_msg.angular.z = (right_speed - left_speed) / 2.0

        self.turtle_pub.publish(twist_msg)
        self.get_logger().info(
            f"Motor Driver -> Left: {left_speed:.2f}, Right: {right_speed:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
