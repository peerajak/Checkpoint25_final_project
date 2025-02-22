FROM ros:noetic-ros-core-focal

# Change the default shell to Bash
SHELL [ "/bin/bash" , "-c" ]

# Install Git
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends git ros-noetic-rviz ros-noetic-compressed-image-transport ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-rqt-image-view ros-noetic-gmapping ros-noetic-navigation ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher ros-noetic-slam-gmapping ros-noetic-dwa-local-planner ros-noetic-joint-state-publisher-gui  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control 

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends build-essential

# Create a Catkin workspace 
RUN source /opt/ros/noetic/setup.bash \
 && mkdir -p /ros1_ws/src \
 && cd /ros1_ws/src \
 && catkin_init_workspace 



# Build the Catkin workspace and ensure it's sourced
RUN source /opt/ros/noetic/setup.bash \
 && cd /ros1_ws \
 && catkin_make
RUN echo "source /ros1_ws/devel/setup.bash" >> ~/.bashrc


# Set the working folder at startup
WORKDIR /ros1_ws

CMD ["/bin/bash"]
