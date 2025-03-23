from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_move_group_launch

## ros2 launch my_moveit_config move_group.launch.py &
## ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py &
## ros2 launch moveit_services planning_sim_scene_service.launch.py &
## ros2 launch moveit_services moveit_sim_service.launch.py &
## ros2 launch my_moveit_config moveit_rviz.launch.py &
# rviz2 -d /ros2_ws/src/Checkpoint25_final_project/rviz/cp25_rviz.rviz



def generate_launch_description():
    declared_arguments = []

    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()
    
    # Move Group Node
    move_group_launch = generate_move_group_launch(moveit_config)

    # moveit_rviz
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config) 

    # my_tf_aruco
    aruco_tf_pub = Node(
        package="my_tf_aruco",
        executable="aruco_realrobot_to_camlink_tf_pub.py",
        output="screen",
    )
    aruco_tf_pub_send_to_tf2_pub = Node(
        package="my_tf_aruco",
        executable="aruco_realrobot_to_camlink_send_to_tf2_pub.py",
        output="screen",
    )
    aruco_tf_pub_tf2_pub = Node(
        package="my_tf_aruco",
        executable="tf2_realrobot_pub_node",
        output="screen",
    )
    aruco_yaw180_tf_pub_tf2_pub = Node(
        package="my_tf_aruco",
        executable="tf2_realrobot_yaw180_aruco_pub_node",
        output="screen",
    )


    # moveit_services
    planning_sim_scene_service_launch = Node(
        package='moveit_services',
        executable='planning_realrobot_scene_service',
        output='screen'
    )
    moveit_realrobot_service_launch = Node(
        #name="moveit_sim_service",
        package="moveit_services",
        executable="moveit_realrobot_service",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': False},
        ],
    )

    # Answer D415 TF position

    D415_link_answer_TF = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['-0.415', '-0.375', '0.31', '1.57', '1.197', '0', 'base_link', 'D415_link'] #Answer (xyz,ypr)
            #arguments = ['-0.415', '-0.375', '0.31', '1.57', '2.7678', '0', 'base_link', 'D415_link_answer'] 
            #arguments = ['-0.415', '-0.375', '0.31', '1.57', '2.7678', '0', 'base_link', 'D415_link_answer'] 
            #arguments = ['-0.415', '-0.375', '0.31', '0.03', '-0.989', '-0.094', '0.11', 'base_link', 'D415_link_answer'] 
            #arguments = ['-0.415', '-0.375', '0.31', '-0.181', '-2.927', '-0.080', 'base_link', 'D415_link_answer'] 
            #arguments = ['-0.415', '-0.375', '0.31', '2.96', '0.6889', '3.0615', 'base_link', 'D415_link_answer'] 
    )

    #rviz2
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("launch_cp25"), "rviz", "cp25_realrobot_rviz.rviz"]
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
        #aruco_tf_pub,
        aruco_tf_pub_send_to_tf2_pub,
        #aruco_tf_pub_tf2_pub,
        aruco_yaw180_tf_pub_tf2_pub,
        planning_sim_scene_service_launch,
        moveit_realrobot_service_launch,
        D415_link_answer_TF,
        rviz_node 

    ])