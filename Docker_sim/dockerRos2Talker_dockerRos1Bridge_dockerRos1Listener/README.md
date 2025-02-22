# Docker Ros1_bridge Talker Listener

https://github.com/ros2/ros1_bridge


## Example 1a: ROS 1 talker and ROS 2 listener

Terminal 1

```
docker build -t test:ros1_bridge -f Dockerfile_Ros1Bridge .
docker run --name r1br2 --rm -it test:ros1_bridge
```

inside docker 

```
    root@8d474171d7d9:/ros2_ws# source $ROS1_INSTALL_PATH/setup.bash
    root@8d474171d7d9:/ros2_ws# roscore

```


Terminal 2

```
docker exec -it r1br2 bash
```

inside docker : Remind that ros1_bridge is a ros2 package

```
    root@8d474171d7d9:/ros2_ws#         cd ../ros1_bridge/
    root@8d474171d7d9:/ros1_bridge#     source ${ROS1_INSTALL_PATH}/setup.bash && source install/setup.bash
    ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions.
    root@8d474171d7d9:/ros1_bridge#     export ROS_MASTER_URI=http://localhost:11311
    root@8d474171d7d9:/ros1_bridge#     ros2 run ros1_bridge dynamic_bridge
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option

```

Terminal 3

```
docker exec -it r1br2 bash
```
inside docker :

```
    root@8d474171d7d9:/ros2_ws#     source ${ROS1_INSTALL_PATH}/setup.bash
    root@8d474171d7d9:/ros2_ws#     rosrun roscpp_tutorials talker

[INFO] [1740107758.177407]: hello world 1740107758.1772978
[INFO] [1740107758.277407]: hello world 1740107758.277299
[INFO] [1740107758.377410]: hello world 1740107758.3773007

```

Terminal 4

```
docker exec -it r1br2 bash
```
inside docker :

```
    root@8d474171d7d9:/ros2_ws#     source ${ROS2_INSTALL_PATH}/setup.bash
    root@8d474171d7d9:/ros2_ws#     ros2 run demo_nodes_cpp listener
[INFO] [1740107591.978605554] [listener]: I heard: [hello world 1740107591.9772522]
[INFO] [1740107592.078926900] [listener]: I heard: [hello world 1740107592.0774307]
[INFO] [1740107592.178903237] [listener]: I heard: [hello world 1740107592.1773846]
[INFO] [1740107592.278886883] [listener]: I heard: [hello world 1740107592.277401]
```

If you see "I heard:" then successful.

### with 4 dockers communicating to each other

Terminal 1

```
docker compose -f docker-compose-ex1a.yml up
```


## Example 1b: ROS 2 talker and ROS 1 listener

Terminal 1

```
docker build -t test:ros1_bridge -f Dockerfile_Ros1Bridge .
docker run --name r1br2 --rm -it test:ros1_bridge
```

inside docker 

```
    root@8d474171d7d9:/ros2_ws# source $ROS1_INSTALL_PATH/setup.bash
    root@8d474171d7d9:/ros2_ws# roscore

```


Terminal 2

```
docker exec -it r1br2 bash
```

inside docker : Remind that ros1_bridge is a ros2 package

```
    root@8d474171d7d9:/ros2_ws#         cd ../ros1_bridge/
    root@8d474171d7d9:/ros1_bridge#     source ${ROS1_INSTALL_PATH}/setup.bash && source install/setup.bash
    ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions.
    root@8d474171d7d9:/ros1_bridge#     export ROS_MASTER_URI=http://localhost:11311
    root@8d474171d7d9:/ros1_bridge#     ros2 run ros1_bridge dynamic_bridge
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option

```


Terminal 3

```
docker exec -it r1br2 bash
```
inside docker : 

```
    root@8d474171d7d9:/ros2_ws#     source ${ROS2_INSTALL_PATH}/setup.bash && ros2 run demo_nodes_py talker
```

Terminal 4

```
docker exec -it r1br2 bash
```
inside docker : 

```
    root@8d474171d7d9:/ros2_ws#     source ${ROS1_INSTALL_PATH}/setup.bash && rosrun roscpp_tutorials listener
```




