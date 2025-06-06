import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


from utils.Robot import Robot
from utils.GoalPlanner import GoalPlanner
import numpy as np



class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')

        # 파라미터 설정
        goal_x = 0.0
        goal_y = -0.5
        goal_theta = 90

        self.robot = Robot(0, 0, 0)  # 초기 pose
        self.planner = GoalPlanner(goal_x, goal_y, goal_theta, self.robot)
        self.planner.setParameter(0.3, 0.8, -0.15)
        self.planner.setDirection()


        self.max_linear_velocity = 0.1   # m/s
        self.max_angular_velocity = 0.4  # rad/s

        self.best_particle_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        # self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer_period = 0.1  # 10ms
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Goal Navigation Node Started.")

    def pose_callback(self, msg):
        pose = msg.pose.pose
        # 2D 좌표 및 오일러 각도로 변환
        x = pose.position.x
        y = pose.position.y

        # Orientation → yaw 변환 (쿼터니언 → 오일러)
        import math
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        # Robot 위치 업데이트
        self.robot.x = x
        self.robot.y = y
        self.robot.theta = self.saturationRad(theta) * 180 / np.pi

    def timer_callback(self):
        # velocity 계산
        v, w = self.planner.calculateVelocity()
        v = self.saturationVelocity(v)
        w = self.saturationAngularVelocity(w)

        # Twist 메시지 생성 및 publish
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f"x={self.robot.x:.2f}, y={self.robot.y:.2f}, theta={self.robot.theta:.2f}")


        self.get_logger().info(f"Publishing cmd_vel: v={v:.2f}, w={w:.2f}")


    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi
    
    def saturationVelocity(self, v):
        if v > self.max_linear_velocity:
            return self.max_linear_velocity
        elif v < -self.max_linear_velocity:
            return -self.max_linear_velocity
        return v

    def saturationAngularVelocity(self, w):
        if w > self.max_angular_velocity:
            return self.max_angular_velocity
        elif w < -self.max_angular_velocity:
            return -self.max_angular_velocity
        return w

def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()