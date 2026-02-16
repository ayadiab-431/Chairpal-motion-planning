import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')

        # Parameters
        self.declare_parameter('wheel_base', 0.6)
        self.wheel_base = self.get_parameter('wheel_base').value

        # Subscriber to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)

        # Publisher for wheel speeds
        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            '/wheel_speeds',
            10)

        self.get_logger().info("Base Controller Started")

    def cmd_vel_callback(self, msg):

        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        wheel_msg = Float32MultiArray()
        wheel_msg.data = [v_left, v_right]

        self.publisher_.publish(wheel_msg)

        self.get_logger().info(
            f"Left: {v_left:.2f} m/s | Right: {v_right:.2f} m/s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
