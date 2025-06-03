import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray
from tf_transformations import euler_from_quaternion
import time

class DifferentialDriveRobot(Node):
    def __init__(self, k_rho=0.6, k_alpha=1, k_beta=0, dt=0.01):
        super().__init__('parking_node')

        self.k_rho = k_rho
        self.k_alpha = k_alpha
        self.k_beta = k_beta
        self.dt = dt

        self.waypoints = []
        self.current_idx = 0
        self.mission = 0
        self.prev_twist = Twist()

        self.last_twist_time = self.get_clock().now()
        self.timeout = 1.0  # 초 단위

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.step, 10)
        self.wp_sub = self.create_subscription(
            PoseArray, '/waypoints', self.waypoint_callback, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.15, self.publish_twist)

    def waypoint_callback(self, msg):
        self.waypoints = []
        for pose in msg.poses:
            x = pose.position.x
            y = pose.position.y
            orientation_q = pose.orientation
            (_, _, yaw) = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])
            self.waypoints.append(([x, y], yaw))
        self.current_idx = 0
        self.mission = 0
        # self.get_logger().info(f"✅ {len(self.waypoints)}개의 웨이포인트 수신")

    def step(self, msg):
        def pp(x): return (x + np.pi) % (2 * np.pi) - np.pi

        if self.current_idx >= len(self.waypoints):
            self.prev_twist.linear.x = 0.0
            self.prev_twist.angular.z = 0.0
            return

        goal_pos, _ = self.waypoints[self.current_idx]
        goal_theta=0

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, psi) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        # dy = goal_pos[0] - y
        # dx = goal_pos[1] - x
        dy =  goal_pos[1]-y
        dx = goal_pos[0] -x
        rho = np.hypot(dy, dx)
        angle_to_goal = np.arctan2(dx, dy)
        self.get_logger().info(f"psi: {psi:.2f}, angle_to_goal: {angle_to_goal:.2f}")
        alpha = pp(angle_to_goal - psi)
        beta = pp(goal_theta - psi)

        forward = True
        # if abs(alpha) > np.pi / 2:
        #     alpha = alpha + np.pi
        #     beta = beta + np.pi

        v = self.k_rho * rho
        if not forward:
            v *= -1
        w = self.k_alpha * alpha + self.k_beta * beta
        # if abs(w)<0.5:
        #     w=0
        # v *= (1 - np.exp(-rho))
        # w *= (1 - np.exp(-rho))
        v = np.clip(v, -0.6,0.6)
        w = np.clip(w, -0.6, 0.6)


        if rho < 1.0 and self.mission == 0:
            self.mission = 1
        if self.mission == 1:
            self.prev_twist.linear.x = 0.0
            w = -self.k_beta * pp(goal_theta - psi)
            w *= 2 / 3
            self.prev_twist.angular.z = w
            if abs(pp(goal_theta - psi)) < 0.1:
                self.mission = 2
        elif self.mission == 2:
            self.prev_twist.linear.x = 0.0
            self.prev_twist.angular.z = 0.0
            self.get_logger().info(f"✅ 웨이포인트 {self.current_idx+1}/{len(self.waypoints)} 도착 완료")
            self.mission = 0
            self.current_idx += 1
        else:
            self.prev_twist.linear.x = v
            self.prev_twist.angular.z = w

        # 🟡 마지막 갱신 시간 저장
        self.last_twist_time = self.get_clock().now()

        self.get_logger().info(f"[{self.current_idx}] x: {x:.2f}, y: {y:.2f}, rho: {rho:.2f}, alpha: {alpha:.2f}, beta: {beta:.2f}")
        self.get_logger().info(f"v: {v:.2f}, w: {w:.2f}")
        # twist_to_publish = Twist()
        # twist_to_publish.linear.x=v
        # twist_to_publish.angular.z=-w
        # self.vel_pub.publish(twist_to_publish)

    def publish_twist(self):
        # 현재 시간과 마지막 갱신 시간 차이 계산
        time_diff = (self.get_clock().now() - self.last_twist_time).nanoseconds * 1e-9

        twist_to_publish = Twist()
        if time_diff <= self.timeout:
            # 최근 명령을 그대로 발행
            twist_to_publish = self.prev_twist
        else:
            # 오래된 명령이면 점진적으로 멈춤
            twist_to_publish.linear.x = self.prev_twist.linear.x * 0.9
            twist_to_publish.angular.z = self.prev_twist.angular.z * 0.9
            self.prev_twist = twist_to_publish  # 상태 유지

        self.vel_pub.publish(twist_to_publish)




def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
