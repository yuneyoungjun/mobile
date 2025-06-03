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
        # 2. mobile 패키지의 plot 노드 실행
        # Node(
        #     package='mobile',
        #     executable='waypoint_pub',
        #     name='plot_node',
        #     output='screen'
        # ),

    ])
