## Example 2: run the bridge and exchange images

Terminal 1

```
docker build -t test:ros1_bridge -f Dockerfile_Ros1Bridge .
```

need to execute docker with display to show image

```
peerajak@ryzen9:~/MyRobotics/gimp$ w
 10:34:50 up  1:41,  1 user,  load average: 0.33, 0.19, 0.44
USER     TTY      FROM             LOGIN@   IDLE   JCPU   PCPU WHAT
peerajak :1       :1               08:53   ?xdm?  10:12   0.00s /usr/libexec/gdm-x-session --run-script env GNOME_SHELL_SESSION_MODE=ubuntu /usr/bin/gnome-session --session=ubuntu
```

The value TTV, :1, is the display number. set DISPLAY environment variable accordingly.

```
export DISPLAY=:1
docker run --rm -it --name r1br2 -e DISPLAY=$DISPLAY  -v /tmp/.X11-unix:/tmp/.X11-unix:ro --runtime=nvidia --gpus all test:ros1_bridge
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
    root@8d474171d7d9:/ros2_ws#         cd ../ros1_bridge/ && source ${ROS1_INSTALL_PATH}/setup.bash && source install/setup.bash && export ROS_MASTER_URI=http://localhost:11311
    ROS_DISTRO was set to 'noetic' before. Please make sure that the environment does not mix paths from different distributions.
    root@8d474171d7d9:/ros1_bridge#     ros2 run ros1_bridge dynamic_bridge
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option
failed to create 2to1 bridge for topic '/rosout' with ROS 2 type 'rcl_interfaces/msg/Log' and ROS 1 type 'rosgraph_msgs/Log': No template specialization for the pair
check the list of supported pairs with the `--print-pairs` option

```

Terminal 3

