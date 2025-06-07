import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import math

class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')

        # Robot 초기 위치
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # deg

        # 목표 위치
        self.goal_x = 3.0
        self.goal_y = -0.2
        self.goal_theta = 90.0  # deg

        # 제어 파라미터
        self.K_rho = 0.3
        self.K_alpha = 0.7
        self.K_beta = -0.15
        self.mode = "1"  # 주행 방향 (전진 or 후진)

        # 속도 제한
        self.max_linear_velocity = 0.1   # m/s
        self.max_angular_velocity = 0.3  # rad/s

        # 방향 설정
        self.setDirection()

        self.best_particle_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Goal Navigation Node Started.")

    def setDirection(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        alpha = self.saturationRad(np.arctan2(dy, dx) - np.deg2rad(self.robot_theta))
        self.mode = "2" if abs(alpha) > np.pi / 2 else "1"

    def pose_callback(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y

        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.robot_x = x
        self.robot_y = y
        self.robot_theta = self.saturationRad(theta) * 180 / np.pi

    def timer_callback(self):
        v, w = self.calculateVelocity()
        v = self.saturationVelocity(v)
        w = self.saturationAngularVelocity(w)

        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f"x={self.robot_x:.2f}, y={self.robot_y:.2f}, theta={self.robot_theta:.2f}")
        self.get_logger().info(f"Publishing cmd_vel: v={v:.2f}, w={w:.2f}")

    def calculateVelocity(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        path_theta = np.arctan2(dy, dx)
        if self.mode == "2":
            path_theta = self.saturationRad(path_theta + np.pi)

        rho = np.hypot(dx, dy)
        alpha = self.saturationRad(path_theta - np.deg2rad(self.robot_theta))
        beta = self.saturationRad(np.deg2rad(self.goal_theta) - path_theta)

        v = self.K_rho * rho
        w = self.K_alpha * alpha + self.K_beta * beta

        if self.mode == "2":
            v = -v

        if rho < 0.15 :
            v = 0
            heading_error = self.saturationRad(np.deg2rad(self.goal_theta) - np.deg2rad(self.robot_theta))
            w = 0.3 * heading_error
            if abs(heading_error) < 5 * np.pi / 180:
                w = 0.0

        return v, w

    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi

    def saturationVelocity(self, v):
        return max(-self.max_linear_velocity, min(self.max_linear_velocity, v))

    def saturationAngularVelocity(self, w):
        return max(-self.max_angular_velocity, min(self.max_angular_velocity, w))


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
