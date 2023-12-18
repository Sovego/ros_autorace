from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bagodelnya_core',
            executable='line_s',
            output='log'
        ),
        Node(
            package='bagodelnya_core',
            executable='camera_s',
            output='log'
        ),
    ])