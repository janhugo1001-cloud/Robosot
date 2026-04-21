import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 地圖路徑（統一在這裡修改）
MAP_YAML = '/home/recoomputer/robot_ws/src/turtlebot4/turtlebot4_navigation/maps/map_urg2.yaml'
MAP_PGM  = '/home/recoomputer/robot_ws/src/turtlebot4/turtlebot4_navigation/maps/map_urg4.pgm'


def generate_launch_description():

    # ─────────────────────────────────────────
    # 1. 雷射啟動
    # 原指令：ros2 launch sllidar_ros2 view_sllidar_a1_launch.py use_sim_time:=false
    # ─────────────────────────────────────────
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('urg_node2'),
            # '/launch/view_sllidar_a1_launch.py'
            '/launch/urg_node2.launch.py'
        ]),
    )
    # ─────────────────────────────────────────
    # 2. 座標轉換（雷射 ↔ 底盤）
    # 原指令：ros2 run tf2_ros static_transform_publisher
    #         --x 0 --y 0 --z 0.1
    #         --roll 3.1415926 --pitch 0 --yaw 1.5707963
    #         --frame-id base_link --child-frame-id laser
    # ─────────────────────────────────────────
    tf_lidar_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=[
            '--x',        '0',
            '--y',        '0',
            '--z',        '0.1',
            '--roll',     '3.1415926',
            '--pitch',    '0',
            '--yaw',      '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser'
        ],
        output='screen'
    )

    # ─────────────────────────────────────────
    # 3. 使用已儲存地圖進行定位（AMCL）
    # 原指令：ros2 launch turtlebot4_navigation localization.launch.py
    #         map:=... use_sim_time:=false
    # ─────────────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot4_navigation'),
            '/launch/localization.launch.py'
        ]),
        launch_arguments={
            'map':          MAP_YAML,
            'use_sim_time': 'false'
        }.items()
    )

    # ─────────────────────────────────────────
    # 4. Nav2 自主導航
    # 原指令：ros2 launch turtlebot4_navigation nav2.launch.py use_sim_time:=false
    # ─────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot4_navigation'),
            '/launch/nav2.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # ─────────────────────────────────────────
    # 5. 發布地圖（map_publisher）
    # 原指令：ros2 run map_publisher map_publisher_node --ros-args
    #         -p map_file:=... -p yaml_file:=...
    # ─────────────────────────────────────────
    map_publisher = Node(
        package='map_publisher',
        executable='map_publisher_node',
        name='map_publisher',
        parameters=[{
            'map_file':  MAP_PGM,
            'yaml_file': MAP_YAML,
        }],
        output='screen'
    )

    # ─────────────────────────────────────────
    # 6. 視覺化（RViz）
    # 原指令：ros2 launch turtlebot4_viz view_robot.launch.py
    # ─────────────────────────────────────────
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot4_viz'),
            '/launch/view_robot.launch.py'
        ])
    )

    # ─────────────────────────────────────────
    # 啟動順序（用 TimerAction 控制延遲）
    # ─────────────────────────────────────────
    return LaunchDescription([

        # 立即啟動：硬體相關
        LogInfo(msg='[1/6] 啟動雷射...'),
        lidar,

        LogInfo(msg='[2/6] 啟動座標轉換...'),
        tf_lidar_to_base,

        # 延遲 3 秒：等雷射就緒後再啟動定位
        TimerAction(period=3.0, actions=[
            LogInfo(msg='[3/6] 啟動定位（AMCL）...'),
            localization,
        ]),

        # 延遲 5 秒：定位啟動後再跑 Nav2
        TimerAction(period=5.0, actions=[
            LogInfo(msg='[4/6] 啟動 Nav2 導航...'),
            nav2,
        ]),

        # 延遲 5 秒：同時啟動地圖發布
        TimerAction(period=5.0, actions=[
            LogInfo(msg='[5/6] 啟動地圖發布...'),
            map_publisher,
        ]),

        # 延遲 7 秒：最後開啟視覺化
        TimerAction(period=7.0, actions=[
            LogInfo(msg='[6/6] 啟動 RViz 視覺化...'),
            rviz,
        ]),

        # echo /cmd_vel
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/cmd_vel'],
            output='screen'
        ),

    ])