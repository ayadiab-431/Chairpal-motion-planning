import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Target Point (مثلاً نقطة أمام الكرسي بـ 3 متر ومائلة شوي)
        self.goal_x = 3.0
        self.goal_y = 1.0
        
        # Current State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Obstacle State
        self.obstacle_detected = False
        
        # Timer للتحديث المستمر للـ Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Goal Navigator has started!')

    def get_yaw_from_quaternion(self, q):
        # تحويل من Quaternion لـ Yaw (زاوية الدوران حول محور Z)
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        
    def scan_callback(self, msg):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return
            
        # التركيز على الجزء الأمامي (نأخذ مثلاً 30 درجة يمين و 30 درجة يسار)
        idx_range = int((30.0 / 360.0) * num_ranges)
        
        front_view = msg.ranges[:idx_range] + msg.ranges[-idx_range:]
        
        # تصفية القراءات غير الصحيحة
        valid_ranges = [r for r in front_view if 0.1 < r < 4.0]
        
        # إذا كان هناك عائق أقرب من 0.7 متر
        if valid_ranges and min(valid_ranges) < 0.7:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def control_loop(self):
        twist = Twist()
        
        # حساب المسافة إلى الهدف
        distance_to_goal = math.sqrt((self.goal_x - self.current_x)**2 + (self.goal_y - self.current_y)**2)
        
        # لو وصلنا للهدف بنسبة خطأ بسيطة
        if distance_to_goal < 0.2:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("🎯 Goal Reached!", throttle_duration_sec=2.0)
            self.cmd_pub.publish(twist)
            return
            
        # لو فيه عائق في الطريق
        if self.obstacle_detected:
            # نلف عشان نتفادى العائق
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # يمكن التعديل للتفاف في عكس اتجاه العائق (بس دي طريقة مبسطة)
            self.get_logger().info("⚠️ Obstacle ahead! Avoiding...", throttle_duration_sec=1.0)
        else:
            # حساب الزاوية المطلوبة للوصول للهدف
            angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
            
            # خطأ الزاوية
            angle_error = angle_to_goal - self.current_yaw
            
            # ضبط زاوية الخطأ لتكون في النطاق [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2.0 * math.pi
            while angle_error < -math.pi:
                angle_error += 2.0 * math.pi
            
            # تحكم مبسط: لو الزاوية كبيرة بنلف في الأول
            if abs(angle_error) > 0.2:
                twist.linear.x = 0.0
                twist.angular.z = 0.5 if angle_error > 0 else -0.5
                self.get_logger().info("🔄 Turning towards goal...", throttle_duration_sec=1.0)
            else:
                # لو الزاوية مظبوطة، نمشي لقدام
                twist.linear.x = 0.3
                twist.angular.z = 0.0
                self.get_logger().info(f"➡️ Moving to goal... Distance remaining: {distance_to_goal:.2f} m", throttle_duration_sec=1.0)
                
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()

if __name__ == '__main__':
    main()
