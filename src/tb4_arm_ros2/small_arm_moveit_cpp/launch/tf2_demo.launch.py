
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config_pkg = get_package_share_directory('small_arm_moveit_config')
    demo_launch_file = os.path.join(moveit_config_pkg, 'launch', 'demo.launch.py')

    return LaunchDescription([

        # 1. 先把原本的 MoveIt demo.launch.py 包進來
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch_file)
        ),

        

        # 2. 手眼標定結果：link6 -> camera

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='link5_to_camera',
            arguments=[
                '0.0657198424207066','0.0555234589837388','-0.08593815259816209',
                '0.06235240401122453','0.07031940325223345','3.1312024787059514', 
                'link5',               # end effector frame
                'camera_frame'         # calibrated camera frame
            ],
            output='screen'
        ),
        # Node(
        #             package='yolo_node',   
        #             executable='yolo_puber_node',
        #             name='object_tf_broadcaster',
        #             output='screen'
        #         ),
        # 3. 你的 tf2 listener
        Node(
            package='small_arm_moveit_cpp',
            executable='tf2_listener',
            name='tf2_listener',
            parameters=[
                {'source_frame': 'base_link'},
                {'target_frame': 'object_frame'}
            ],
            output='screen'
        ),
        
    ])