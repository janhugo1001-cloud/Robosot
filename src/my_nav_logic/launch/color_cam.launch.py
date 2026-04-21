from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'align_depth.enable': 'true'
        }.items()
    )

    color_cam_node = Node(
        package='my_nav_logic',
        executable='color_cam',
        name='color_cam',
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        color_cam_node,
    ])