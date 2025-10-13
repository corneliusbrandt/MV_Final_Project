from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
