#! /bin/bash 

source /ros2_ws/install/setup.bash 

ros2 launch the_construct_office_gazebo starbots_ur3e.launch.xml &
ros2 launch my_moveit_config move_group.launch.py &
ros2 launch my_moveit_config moveit_rviz.launch.py


