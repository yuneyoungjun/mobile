import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
from rclpy.qos import qos_profile_sensor_data


class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.goal_x = 3
        self.goal_y = 0
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


        # 플롯 초기화
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

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_title("Robot Pose & LiDAR")

        # 현재 위치 점 표시
        self.ax.scatter(self.robot_x, self.robot_y, c='b', marker='o', label='Robot')

        # 방향 화살표 표시 (로봇 포즈 시각화)
        arrow_length = 0.5
        dx = arrow_length * np.cos(self.robot_theta)
        dy = arrow_length * np.sin(self.robot_theta)
        self.ax.arrow(self.robot_x, self.robot_y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        # 목표 위치
        self.ax.scatter(self.goal_x, self.goal_y, c='r', marker='x', label='Goal')

        # 라이다 데이터 (전역 좌표계로 변환)
        if len(self.laser_ranges) > 0 and len(self.laser_angles) > 0:
            # 유효한 값만 필터링
            valid_indices = np.isfinite(self.laser_ranges)
            valid_ranges = self.laser_ranges[valid_indices]
            valid_angles = self.laser_angles[valid_indices]

            # 로컬 좌표계에서의 위치 계산
            local_x = valid_ranges * np.cos(valid_angles)
            local_y = valid_ranges * np.sin(valid_angles)

            # 전역 좌표계로 변환 (회전 + 이동)
            global_x = self.robot_x + (np.cos(self.robot_theta) * local_x - np.sin(self.robot_theta) * local_y)
            global_y = self.robot_y + (np.sin(self.robot_theta) * local_x + np.cos(self.robot_theta) * local_y)

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
