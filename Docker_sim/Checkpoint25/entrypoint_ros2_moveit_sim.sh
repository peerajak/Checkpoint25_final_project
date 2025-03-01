#! /bin/bash 

source /ros2_ws/install/setup.bash 

ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml &
ros2 launch my_moveit_config move_group.launch.py &
ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py &
ros2 launch moveit_services planning_sim_scene_service.launch.py &
ros2 launch moveit_services moveit_sim_service.launch.py &
ros2 launch my_moveit_config moveit_rviz.launch.py &
rviz2 -d /ros2_ws/src/Checkpoint25_final_project/rviz/cp25_rviz.rviz




