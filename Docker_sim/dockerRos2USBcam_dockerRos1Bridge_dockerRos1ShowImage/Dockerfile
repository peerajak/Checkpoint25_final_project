FROM ros:humble-ros-core-jammy

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Git
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard  ros-humble-teleop-twist-joy ros-humble-rviz2 ros-humble-xacro ros-humble-nav2* ros-humble-urdf python3-colcon-common-extensions cmake pkg-config && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends build-essential

# Create a Catkin workspace and clone Tortoisebot repos
RUN source /opt/ros/humble/setup.bash 
RUN mkdir -p /ros2_ws/src 
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc


# Set the working folder at startup
WORKDIR /ros2_ws



RUN apt-get update &&  DEBIAN_FRONTEND=noninteractive apt-get install -y vim ros-humble-image-tools apt install -y ros-humble-rqt-image-view
# Gazenp and RVIZ slam
#ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True"]
CMD ["/bin/bash"]
