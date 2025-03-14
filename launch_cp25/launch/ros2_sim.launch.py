from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

## ros2 launch my_moveit_config move_group.launch.py &
## ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py &
## ros2 launch moveit_services planning_sim_scene_service.launch.py &
## ros2 launch moveit_services moveit_sim_service.launch.py &
## ros2 launch my_moveit_config moveit_rviz.launch.py &
# rviz2 -d /ros2_ws/src/Checkpoint25_final_project/rviz/cp25_rviz.rviz



def generate_launch_description():
    declared_arguments = []

    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()   
    
    # Move Group Node
    move_group_launch = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
            {"publish_robot_description_semantic": True},
            {"use_sim_time": True},
        ],
    )

    # moveit_rviz
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config) 

    # my_tf_aruco
    aruco_tf_pub = Node(
        package="my_tf_aruco",
        executable="aruco_to_camlink_tf_pub.py",
        output="screen",
    )

    # moveit_services
    planning_sim_scene_service_launch = Node(
        package='moveit_services',
        executable='planning_sim_scene_service',
        output='screen'
    )
    moveit_sim_service_launch = Node(
        #name="moveit_sim_service",
        package="moveit_services",
        executable="moveit_sim_service",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    #rviz2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("launch_cp25"), "rviz", "cp25_rviz.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        move_group_launch ,
        moveit_rviz_launch,
        aruco_tf_pub,
        planning_sim_scene_service_launch,
        moveit_sim_service_launch,
        rviz_node 

    ])