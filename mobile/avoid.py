import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
import time


class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')

        # Robot 상태
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # deg
        self.obs_time=0.0

        # 목표 및 웨이포인트
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.v=0
        self.w=0
        self.goal_theta =  -179.0
        self.waypoint = [[0.0, 0.0], [2.1, 0.8], [3.83, -1.02], [5.75, 0.45], [5.74,0.0],[0.0, 0.0]]
        # self.waypoint=[[0.0,0.0],[2.0,1.0],[4.0, -1.0],[6.0,0.0],[0.0,0.0] ]
        self.waypoint=[[0.0,0.0],[2.2,0.72],[3.84, -1.0],[5.98,0.47],[4.0,-0.19],[0,0]]
        self.waypoint=[[0.0,0.0],[2.55,0.76],[4.05, -1.1],[6.0,0.46],[0,-0.2]]
        self.range = 0.4

        # 장애물 회피
        self.obs_range = 0.55
        self.scan_avg = 0.0
        self.obs = 0

        # 타이머 관리
        self.waypoint_change_time = time.time()

        # 제어 파라미터
        self.K_rho = 0.3
        self.K_alpha = 0.7
        self.K_beta = -0.0
        self.mode = "1"

        self.max_linear_velocity = 0.1
        self.max_angular_velocity = 0.5

        self.setDirection()

        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos_profile_sensor_data)
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
        if self.waypoint:
            self.goal_x = self.waypoint[0][0]
            self.goal_y = self.waypoint[0][1]

        self.v, self.w = self.calculateVelocity()
        self.v = self.saturationVelocity(self.v)
        self.w = self.saturationAngularVelocity(self.w)

        twist = Twist()
        twist.linear.x = float(self.v)
        twist.angular.z = float(self.w)
        self.cmd_vel_pub.publish(twist)

    def calculateVelocity(self):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        path_theta = np.arctan2(dy, dx)
        if self.mode == "2":# self.waypoint=[[0.0,0.0],[2.0,1.0],[4.0, -1.0],[6.0,0.0],[0.0,0.0] ]
            path_theta = self.saturationRad(path_theta + np.pi)

        rho = np.hypot(dx, dy)
        alpha = self.saturationRad(path_theta - np.deg2rad(self.robot_theta))
        beta = self.saturationRad(np.deg2rad(self.goal_theta) - path_theta)

        self.v = self.K_rho * rho
        self.w = self.K_alpha * alpha + self.K_beta * beta

        # if self.mode == "2":
        #     v = -v

        if rho < self.range:
            if self.waypoint:
                self.waypoint.pop(0)
                # self.obs = 0
                self.waypoint_change_time = time.time()  # 현재 시간 저장
                self.goal_theta = 0.0  # 초기화
                self.setDirection()

        return self.v, self.w

    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi

    def saturationVelocity(self, v):
        return max(-self.max_linear_velocity, min(self.max_linear_velocity, v))

    def saturationAngularVelocity(self, w):
        return max(-self.max_angular_velocity, min(self.max_angular_velocity, w))

    def laser_callback(self, scan: LaserScan):
        # 웨이포인트 변경 후 3초간은 장애물 감지 무시
        if time.time() - self.waypoint_change_time < 7.0:
            self.obs = 0
            return
        if time.time()-self.obs_time<13.5:
            return

        # self.scan_avg = 0
        count = 0
        self.obs = 0
        self.setDirection()
        self.scan_avg=0
        avg1=0
        avg2=0
        # self.obs=0
        self.goal_theta=179
        for i in range(8):
            if scan.ranges[i]==0:
                scan.ranges[i]=7
            avg1+=scan.ranges[i]/16
            avg2+=scan.ranges[349+i]/16
            self.scan_avg=avg1+avg2

        # print(self.scan_avg)
        if 0<self.scan_avg<self.obs_range:
                self.v=self.v/2
                # if len(self.waypoint)%2==0:
                #     self.obs=1
                # if len(self.waypoint)%2==1:
                #     self.obs=2
                # # if len(self.waypoint)==1:
                # #     self.obs=3
                if(len(self.waypoint))==1:
                    if time.time() - self.waypoint_change_time < 3.0:
                        self.v=0
                    if avg1>avg2:
                        self.obs=2
                    else:
                        self.obs=2
        if(len(self.waypoint))==1:
            self.K_beta=-0.25
        print(self.scan_avg)
        if self.obs==1:
            self.K_beta=-0.25
            self.goal_theta=45
        elif self.obs==2:
            self.K_beta=-0.25
            self.goal_theta=45
            self.obs_time = time.time()
        print(self.obs)
        if(len(self.waypoint))==0:
            self.v=0
            self.w=0
        else:
            print(self.waypoint[0])
        # elif self.obs==3:
        #     self.w=self.w*2

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
