from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_services',
            executable='planning_sim_scene_service',
            output='screen'),
    ])