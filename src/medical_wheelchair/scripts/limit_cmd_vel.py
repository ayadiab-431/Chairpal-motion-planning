import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelLimiter(Node):
    def __init__(self):
        super().__init__('cmd_vel_limiter')
        # Subscribes to the raw (unfiltered) velocity commands
        self.sub = self.create_subscription(Twist, '/cmd_vel_raw', self.cb, 10)
        # Publishes the filtered/limited velocity commands
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.max_lin = 0.15   # m/s (Maximum linear velocity)
        self.max_ang = 0.4    # rad/s (Maximum angular velocity)
        self.get_logger().info(f'CmdVelLimiter started: Max Lin={self.max_lin}, Max Ang={self.max_ang}')

    def cb(self, msg: Twist):
        # Create a new message to avoid modifying the original one in place if needed
        limited_msg = Twist()
        # Clamp linear.x
        limited_msg.linear.x = max(min(msg.linear.x, self.max_lin), -self.max_lin)
        # Clamp angular.z
        limited_msg.angular.z = max(min(msg.angular.z, self.max_ang), -self.max_ang)
        
        # Ensure other fields are zeroed out (since this is a diff-drive robot)
        limited_msg.linear.y = 0.0
        limited_msg.linear.z = 0.0
        limited_msg.angular.x = 0.0
        limited_msg.angular.y = 0.0
        
        self.pub.publish(limited_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
