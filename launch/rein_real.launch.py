from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='mobile',
            executable='particle_filter',
            name='particle_filter_node',
            output='screen'
        ),
        Node(
            package='mobile',
            executable='waypoint_pub',
            name='waypoint_pub_node',
            output='screen'
        ),

        # 4. mobile 패키지의 parcking 노드 실행 (오타가 "parcking" 맞는지 확인 필요)
        Node(
            package='mobile',
            executable='rein',
            name='rein_node',
            output='screen'
        ),
    ])
