from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_navigator',
            executable='pose_navigator',
            name='pose_navigator',
            output='screen'
        )
    ]) 