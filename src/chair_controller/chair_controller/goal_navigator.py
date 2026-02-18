import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


class GoalNavigator(Node):

    def __init__(self):
        super().__init__('goal_navigator')

        # Subscribe to goal from Rviz "2D Goal Pose"
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Subscribe to current position
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Current position
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Goal
        self.goal_x = None
        self.goal_y = None
        self.has_goal = False

        # Control parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.distance_tolerance = 0.15
        self.angle_tolerance = 0.1

        # Timer for control loop at 10Hz
        self.create_timer(0.1, self.navigate)

        self.get_logger().info('Goal Navigator Ready! Use "2D Goal Pose" in Rviz to set a target.')

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.has_goal = True
        self.get_logger().info(
            f'New Goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.th = math.atan2(siny, cosy)

    def navigate(self):
        if not self.has_goal:
            return

        # Calculate distance and angle to goal
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)
        angle_to_goal = math.atan2(dy, dx)

        # Angle error (normalized to [-pi, pi])
        angle_error = angle_to_goal - self.th
        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi
        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        twist = Twist()

        # Reached goal?
        if distance < self.distance_tolerance:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.has_goal = False
            self.get_logger().info(
                f'Goal Reached! ({self.goal_x:.2f}, {self.goal_y:.2f})')
            return

        # First rotate to face goal, then drive
        if abs(angle_error) > self.angle_tolerance:
            # Rotate only
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed * (1.0 if angle_error > 0 else -1.0)
        else:
            # Drive forward + small angular correction
            twist.linear.x = min(self.linear_speed, distance)
            twist.angular.z = angle_error * 2.0  # proportional correction

        self.cmd_pub.publish(twist)
        self.get_logger().info(
            f'Navigating: dist={distance:.2f}, angle_err={math.degrees(angle_error):.1f}°')

    def destroy_node(self):
        # Stop the robot before shutting down
        twist = Twist()
        self.cmd_pub.publish(twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
