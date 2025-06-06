import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped, Twist
from tf_transformations import quaternion_from_euler, euler_from_quaternion

# 🚀 **웨이포인트 퍼블리셔 노드**
class WaypointPublisher(Node):
    def __init__(self, waypoints):
        super().__init__('waypoint_publisher')
        self.waypoints = waypoints
        self.waypoint_pub = self.create_publisher(PoseArray, '/waypoints', 10)

        # ✅ 일정 간격으로 웨이포인트 퍼블리시
        self.timer = self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        for wp in self.waypoints:
            pose = Pose()
            pose.position.x = wp[0]
            pose.position.y = wp[1]
            pose.position.z = 0.0

            qx, qy, qz, qw = quaternion_from_euler(0, 0, 0)  # 방향 제거
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw

            msg.poses.append(pose)

        self.get_logger().info("Publishing waypoints...")
        self.waypoint_pub.publish(msg)


# 🚀 **웨이포인트를 따라가는 로봇 노드**
class DifferentialDriveRobot(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.waypoints = []  # 웨이포인트 저장
        self.current_waypoint_idx = 0  # 현재 목표 웨이포인트 인덱스
        self.k_rho = 3.0
        self.k_alpha = 8.0
        self.k_beta = 0
        self.mission_complete = False  # 모든 웨이포인트 완료 여부

        self.create_subscription(PoseArray, '/waypoints', self.waypoints_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.step, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.prev_twist = Twist()
        self.prev_twist.linear.x = 0.0
        self.prev_twist.angular.z = 0.0
        self.timer = self.create_timer(0.05, self.publish_twist)

    def waypoints_callback(self, msg):
        self.waypoints = [[pose.position.x, pose.position.y] for pose in msg.poses]
        self.get_logger().info("Waypoints received!")

    def step(self, msg):
        if self.mission_complete or not self.waypoints:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, psi) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

        # ✅ 현재 목표 웨이포인트 가져오기
        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]
        dx = goal_x - x
        dy = goal_y - y
        rho = np.hypot(dx, dy)
        angle_to_goal = np.arctan2(dy, dx)

        alpha = angle_to_goal - psi

        forward = True
        if abs(alpha) > np.pi / 2:
            alpha += np.pi
            forward = not forward

        v = self.k_rho * rho * np.exp(-rho)
        if not forward:
            v *= -1

        w = self.k_alpha * alpha

        scale_v = 0.1
        scale_w = 0.1
        w *= scale_w * (1 - np.exp(-rho))
        v *= scale_v * (1 - np.exp(-rho))

        self.get_logger().info(f"Waypoint {self.current_waypoint_idx}: x={x}, y={y}, psi={psi}, alpha={alpha}")
        self.get_logger().info(f"v={v}, w={w}")

        # ✅ 속도 업데이트
        self.prev_twist.linear.x = v
        self.prev_twist.angular.z = w

        # ✅ 웨이포인트 도착 검사
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            self.get_logger().info(f"Waypoint {self.current_waypoint_idx} reached! 🚀")
            self.current_waypoint_idx += 1  # 다음 웨이포인트로 이동

            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints completed. Stopping robot.")
                self.mission_complete = True
                self.prev_twist.linear.x = 0.0
                self.prev_twist.angular.z = 0.0

    def publish_twist(self):
        self.vel_pub.publish(self.prev_twist)

def main(args=None):
    rclpy.init(args=args)

    # ✅ 이동할 웨이포인트 리스트
    waypoints = [[1.0, 1.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]

    # ✅ 두 개의 노드 실행
    wp_publisher = WaypointPublisher(waypoints)
    wp_follower = DifferentialDriveRobot()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wp_publisher)
    executor.add_node(wp_follower)

    executor.spin()

    wp_publisher.destroy_node()
    wp_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
