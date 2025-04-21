import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='moveit_services',
#             executable='moveit_sim_hole_service',
#             output='screen'),
#     ])

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # MoveItCpp demo executable
    moveit_sim_hole_service_node = Node(
        #name="moveit_sim_hole_service",
        package="moveit_services",
        executable="moveit_sim_hole_service",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        [moveit_sim_hole_service_node]
    )