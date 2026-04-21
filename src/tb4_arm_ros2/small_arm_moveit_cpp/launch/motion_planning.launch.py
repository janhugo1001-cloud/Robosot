from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="small_arm", package_name="small_arm_moveit_config"
        )
        .robot_description(file_path="config/small_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/small_arm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("small_arm_moveit_cpp")
            + "/config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )
    catch_node = Node(
        name="catch",
        package="small_arm_moveit_cpp",
        executable="catch",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    catch_grasp_node = Node(
        name="catch_grasp",
        package="small_arm_moveit_cpp",
        executable="catch_grasp",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    run_catch_grasp_after_catch = RegisterEventHandler(
        OnProcessExit(
            target_action=catch_node,
            on_exit=[catch_grasp_node],
        )
    )


    return LaunchDescription(
        [
            catch_node,
            run_catch_grasp_after_catch,
        ]
    )
