import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.waypoints = [[0.0,0.0]]
        self.current_waypoint_idx = 0
        self.laser_ranges = []
        self.laser_angles = []
        self.first=0
        self.initial_x=0
        self.initial_x=0

        # ROS 2 구독자
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            qos_profile_sensor_data
        )

        # ROS 2 퍼블리셔 (클릭으로 추가된 웨이포인트 지속 발행)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/waypoints', 10)

        # Matplotlib 시각화
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.mpl_connect('button_press_event', self.onclick)  # ✅ 마우스 클릭 이벤트 등록

        # 타이머: 0.1초마다 업데이트
        self.timer = self.create_timer(0.1, self.timer_callback)

    def pose_callback(self, msg):
        self.first+=1
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

    def onclick(self, event):
        """ ✅ 마우스 클릭 시 좌표를 웨이포인트로 추가 """
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata
        self.waypoints.append([x, y])
        if self.current_waypoint_idx is None:
            self.current_waypoint_idx = len(self.waypoints) - 1
        self.get_logger().info(f"Added waypoint: ({x:.2f}, {y:.2f})")

    def timer_callback(self):
        self.update_plot()
        self.publish_waypoints()

    def publish_waypoints(self):
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.2
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime.sec = 0  # 0이면 무한

            marker_array.markers.append(marker)

        self.waypoint_pub.publish(marker_array)

    def update_plot(self):
        self.ax.clear()
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_title("Robot Pose & LiDAR")

        # 로봇 위치 및 방향
        self.ax.scatter(self.robot_x, self.robot_y, c='b', marker='o', label='Robot')
        dx = 0.5 * np.cos(self.robot_theta)
        dy = 0.5 * np.sin(self.robot_theta)
        self.ax.arrow(self.robot_x, self.robot_y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        # 웨이포인트 표시
        if self.waypoints:
            waypoint_xs, waypoint_ys = zip(*self.waypoints)
            self.ax.scatter(waypoint_xs, waypoint_ys, c='r', marker='x', label='Waypoints')

            if self.current_waypoint_idx is not None and 0 <= self.current_waypoint_idx < len(self.waypoints):
                cx, cy = self.waypoints[self.current_waypoint_idx]
                self.ax.scatter(cx, cy, c='lime', marker='*', s=200, label='Current Target')
                dist = np.hypot(self.robot_x - cx, self.robot_y - cy)
                if dist < 0.5:
                    self.waypoints.pop(0)
                    # self.current_waypoint_idx += 1
                    if self.current_waypoint_idx >= len(self.waypoints):
                        self.get_logger().info("All waypoints reached!")
                        self.current_waypoint_idx = None

        # LiDAR 데이터 시각화
        if len(self.laser_ranges) > 0 and len(self.laser_angles) > 0:
            valid = np.isfinite(self.laser_ranges)
            r = self.laser_ranges[valid]
            theta = self.laser_angles[valid]
            lx = r * np.cos(theta)
            ly = r * np.sin(theta)

            gx = self.robot_x + (np.cos(self.robot_theta) * lx - np.sin(self.robot_theta) * ly)
            gy = self.robot_y + (np.sin(self.robot_theta) * lx + np.cos(self.robot_theta) * ly)

            self.ax.scatter(gx, gy, c='g', marker='.', alpha=0.5, label='LiDAR')

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
