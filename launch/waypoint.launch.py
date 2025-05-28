from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_world_launch = os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')

    return LaunchDescription([
        # 1. Gazebo 환경 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_world_launch)
        ),

        # 2. mobile 패키지의 plot 노드 실행
        Node(
            package='mobile',
            executable='plot',
            name='plot_node',
            output='screen'
        ),

        # 3. mobile 패키지의 particle_filter 노드 실행
        Node(
            package='mobile',
            executable='particle_filter',
            name='particle_filter_node',
            output='screen'
        ),

        # 4. mobile 패키지의 parcking 노드 실행 (오타가 "parcking" 맞는지 확인 필요)
        Node(
            package='mobile',
            executable='waypoint',
            name='waypoint_node',
            output='screen'
        ),
    ])
