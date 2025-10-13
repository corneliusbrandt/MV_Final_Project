from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node

def generate_launch_description():
    start_rviz = LaunchConfiguration('start_rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
            description='Start rviz alongside gazebo'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_manipulation_moveit_config'),
                    'launch',
                    'move_group.launch.py'
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_manipulation_moveit_config'),
                    'launch',
                    'servo.launch.py'
                )
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('turtlebot3_manipulation_gazebo'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            launch_arguments={'start_rviz': start_rviz}.items()
        ),
        Node(
            package='gesturebot_gestures',
            executable='gesture_detector',
            output='screen'
        ),
        Node(
            package='gesturebot_manipulation',
            executable='manipulation_handler',
            output='screen'
        )
    ])
