#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import math

def create_pose(x, y, yaw_deg, frame_id='map'):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    return pose

def calc_yaw_from_points(x1, y1, x2, y2):
    return math.degrees(math.atan2(y2 - y1, x2 - x1))

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # 초기 위치 설정 (AMCL 작동을 위해 필요)
    initial_pose = create_pose(0.0, 0.0, 0.0)
    navigator.setInitialPose(initial_pose)

    # Nav2 활성화 대기
    navigator.waitUntilNav2Active()

    # 좌표만 정의 (yaw는 자동 계산)
    waypoint_coords = [
        (-2.0, 1.0),
        (-4.0, -1.0),
        (-6.0, 1.0),
        (0.0, 0.0)
    ]

    waypoints = []
    for i in range(len(waypoint_coords)):
        x, y = waypoint_coords[i]
        if i < len(waypoint_coords) - 1:
            next_x, next_y = waypoint_coords[i + 1]
            yaw = calc_yaw_from_points(x, y, next_x, next_y)
        else:
            yaw = 0.0  # 마지막 지점은 원하는 방향으로
        waypoints.append(create_pose(x, y, yaw))

    # 경로 주행 시작
    navigator.followWaypoints(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        print("이동 중... 현재 상태:", feedback)

    result = navigator.getResult()
    print("주행 완료:", result)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
