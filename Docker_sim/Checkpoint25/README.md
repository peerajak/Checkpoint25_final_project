docker build -t ros2_moveit_sim:v1 -f ./dockerfile_ros2_moveit_sim .
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

