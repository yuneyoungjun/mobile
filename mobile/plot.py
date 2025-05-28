import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data
class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        self.robot_x = 0.5
        self.robot_y = 0.5
        self.robot_theta = 0.0
        self.waypoints = []
        self.current_waypoint_idx = None  # ✅ 현재 타겟 웨이포인트 인덱스
        self.laser_ranges = []
        self.laser_angles = []

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(PoseArray, '/waypoints', self.waypoints_callback, 10)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.1, self.update_plot)

    def pose_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        (_, _, self.robot_theta) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

    def laser_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        self.laser_angles = angle_min + np.arange(len(self.laser_ranges)) * angle_increment

    def waypoints_callback(self, msg):
        self.waypoints = [[pose.position.x, pose.position.y] for pose in msg.poses]
        self.current_waypoint_idx = 0 if self.waypoints else None
        self.get_logger().info(f"Received {len(self.waypoints)} waypoints!")

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_title("Robot Pose & LiDAR")

        # 로봇 위치 및 방향 표시
        self.ax.scatter(self.robot_x, self.robot_y, c='b', marker='o', label='Robot')
        arrow_length = 0.5
        dx = arrow_length * np.cos(self.robot_theta)
        dy = arrow_length * np.sin(self.robot_theta)
        self.ax.arrow(self.robot_x, self.robot_y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        # 웨이포인트 전체 표시
        if self.waypoints:
            waypoint_xs, waypoint_ys = zip(*self.waypoints)
            self.ax.scatter(waypoint_xs, waypoint_ys, c='r', marker='x', label='Waypoints')

            # 현재 타겟 웨이포인트 표시
            if self.current_waypoint_idx is not None and 0 <= self.current_waypoint_idx < len(self.waypoints):
                cx, cy = self.waypoints[self.current_waypoint_idx]
                self.ax.scatter(cx, cy, c='lime', marker='*', s=200, label='Current Target')

                # 도달하면 다음 웨이포인트로 이동
                dist = np.hypot(self.robot_x - cx, self.robot_y - cy)
                if dist < 0.3:  # 도달 기준 거리
                    self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.waypoints):
                        self.get_logger().info("All waypoints reached!")
                        self.current_waypoint_idx = None

        # 라이다 시각화
        if len(self.laser_ranges) > 0 :
            valid_indices = np.isfinite(self.laser_ranges)
            valid_ranges = self.laser_ranges[valid_indices]
            valid_angles = self.laser_angles[valid_indices]

            local_x = valid_ranges * np.cos(valid_angles)
            local_y = valid_ranges * np.sin(valid_angles)

            # global_x = self.robot_x + (np.cos(self.robot_theta) * local_x + np.sin(self.robot_theta) * local_y)
            # global_y = self.robot_y + (np.sin(self.robot_theta) * local_x + np.cos(self.robot_theta) * local_y)



            global_x = (np.cos(self.robot_theta) * local_x + np.sin(self.robot_theta) * local_y)
            global_y = (np.sin(self.robot_theta) * local_y + np.cos(self.robot_theta) * local_x)


            self.ax.scatter(global_x, global_y, c='g', marker='.', alpha=0.5, label='LiDAR')

        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = LivePlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
