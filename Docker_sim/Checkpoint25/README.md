docker build -t ros2_moveit_sim:v1 -f ./dockerfile_ros2_moveit_sim .
docker build -t ubuntu2004_webapp:v1 -f ./dockerfile_webapp .
docker context use default
xhost +local:root
docker run --rm -it --name ros2_sim -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --runtime=nvidia --gpus all ros2_moveit_sim:v1 bash
docker exec -it ros2_sim bash


Controller Manager sometime takes long time, 3 mins, to start, expecially first time. Remember to wait for controller manager node to start before conclude that the controller manager is broken.


Terminal 1

docker run --rm -it --name ros2_sim -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --runtime=nvidia --gpus all ros2_moveit_sim:v1 

Terminal 2

docker exec -it ros2_sim bash

inside docker

```
root@779a401aa0dd:/ros2_ws# source install/setup.bash; ros2 launch my_moveit_config move_group.launch.py
```

Terminal 3

docker exec -it ros2_sim bash

inside docker

```
root@779a401aa0dd:/ros2_ws# source install/setup.bash; ros2 launch my_moveit_config moveit_rviz.launch.py
```

Terminal 4

docker exec -it ros2_sim bash

inside docker

```
root@779a401aa0dd:/ros2_ws# source install/setup.bash; ros2 launch moveit2_scripts move_sim_arm_to_show_aruco_trajectory.launch.py
```

Terminal 5

docker exec -it ros2_sim bash

inside docker

```
source install/setup.bash; ros2 run my_tf_aruco aruco_to_camlink_tf_pub.py
```

Terminal 6

docker exec -it ros2_sim bash

inside docker

```
cd  /ros2_ws/src/Checkpoint25_final_project/
rviz2 -d rviz/cp25_rviz.rviz
```



## How to create web bridge

step 1. From ros1_bridge docker, create ros1_and_web_bridge docker, and install the following packages
    - ROS1 rosbridge_server
    - ROS1 web_video_server
    - ROS1 course_web_dev

```
docker build -t ros1_and_web_bridge:v1 -f ./Dockerfile_Ros1_Web_Bridge .
```

step 2. ROS2 camera topic --> ros1_bridge (topic selection) --> ROS1 rosbridge, web_video_server --> WEBPage {connect, show camera images on web}

step 3. ROS2 3d robot model --> ros1_bridge (topic selection) --> ROS1 rosbridge --> WEBPage {Robot model}

step 4. Set Web's Robot model state according to joint_state, robot_state topics.]

step 5. Aruco detector, and aruco position, orientation in text, and aruco axes in the video image

(optional step 5): Show pointclound and aruco axes.
