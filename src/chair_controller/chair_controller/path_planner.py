import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import heapq

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_planner')

        # Publisher للحركة
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber للObstacle Detector
        self.obstacle_sub = self.create_subscription(
            Bool, '/obstacle_detected', self.obstacle_callback, 10)

        # Goal و position
        self.goal = (8, 8)
        self.position = (0, 0)

        self.obstacle_detected = False

        # خريطة 9x9
        self.map = [
            [0,0,0,0,0,0,0,0,0],
            [0,1,1,1,0,1,1,1,0],
            [0,0,0,1,0,0,0,1,0],
            [0,1,0,0,0,1,0,0,0],
            [0,1,0,1,0,1,0,1,0],
            [0,0,0,1,0,0,0,1,0],
            [0,1,0,0,0,1,0,0,0],
            [0,1,1,1,0,1,1,1,0],
            [0,0,0,0,0,0,0,0,0],
        ]

        self.timer = self.create_timer(2.0, self.navigate)

    # ---------------- A* Algorithm ----------------
    def heuristic(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def astar(self, start, goal):
        rows = len(self.map)
        cols = len(self.map[0])

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start:0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            neighbors = [(1,0),(-1,0),(0,1),(0,-1)]

            for dx,dy in neighbors:
                nx,ny = current[0]+dx, current[1]+dy

                if 0<=nx<rows and 0<=ny<cols and self.map[nx][ny]==0:
                    tentative = g_score[current]+1

                    if (nx,ny) not in g_score or tentative<g_score[(nx,ny)]:
                        g_score[(nx,ny)] = tentative
                        f = tentative + self.heuristic((nx,ny),goal)
                        heapq.heappush(open_set,(f,(nx,ny)))
                        came_from[(nx,ny)] = current

        return []

    # ---------------- Navigation ----------------
    def navigate(self):
        if self.obstacle_detected:
            # فيه obstacle → وقف الحركة
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("Obstacle detected! Waiting and recalculating...")
            return

        # حساب المسار للنقطة التالية
        path = self.astar(self.position, self.goal)
        if not path:
            self.get_logger().info("No Path Found!")
            return

        next_point = path[0]

        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        self.position = next_point
        self.get_logger().info(f"Moving to {next_point}")

    # ---------------- Obstacle Callback ----------------
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        if msg.data:
            self.get_logger().info("Obstacle detected signal received!")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
