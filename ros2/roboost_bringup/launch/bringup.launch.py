"""Host side of the robot: URDF/TF, odometry relay, optionally RViz and SLAM.

The micro-ROS agent runs as its own container (see .devcontainer/docker-compose.yml).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = Path(get_package_share_directory('roboost_bringup'))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz'),
        DeclareLaunchArgument('slam', default_value='false', description='Start slam_toolbox mapping'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': (share / 'urdf' / 'roboost.urdf').read_text()}],
        ),
        Node(package='roboost_bringup', executable='odom_tf'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'roboost.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={'slam_params_file': str(share / 'config' / 'slam_toolbox.yaml')}.items(),
            condition=IfCondition(LaunchConfiguration('slam')),
        ),
    ])
