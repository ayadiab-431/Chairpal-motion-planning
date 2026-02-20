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

        # Publisher للـ Simulation (Gazebo Wheelchair)
        self.sim_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.get_logger().info("Motor Driver ready 👑")

    def speed_callback(self, msg):
        # التأكد من استلام 6 سرعات
        if len(msg.data) == 6:
            front_left_speed = msg.data[0]
            middle_left_speed = msg.data[1]
            rear_left_speed = msg.data[2]
            
            front_right_speed = msg.data[3]
            middle_right_speed = msg.data[4]
            rear_right_speed = msg.data[5]

            # في المحاكاة (الـ diff-drive المدمج في Gazebo) بنحتاج نرجع السرعة المجمعة (متوسط اليمين واليسار)
            avg_left_speed = (front_left_speed + middle_left_speed + rear_left_speed) / 3.0
            avg_right_speed = (front_right_speed + middle_right_speed + rear_right_speed) / 3.0

            # تحويل السرعات لأمر Twist لـ Gazebo
            twist_msg = Twist()
            twist_msg.linear.x = (avg_left_speed + avg_right_speed) / 2.0
            twist_msg.angular.z = (avg_right_speed - avg_left_speed) / 2.0

            self.sim_pub.publish(twist_msg)
            self.get_logger().info(
                f"Simulating 6 Motors -> Avg L: {avg_left_speed:.2f}, Avg R: {avg_right_speed:.2f}"
            )
        else:
            self.get_logger().warn("Received incorrect number of wheel speeds. Expected 6.")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
