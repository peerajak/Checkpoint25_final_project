FROM ubuntu:jammy
ENV DEBIAN_FRONTEND=noninteractive
RUN locale  # check for UTF-8

RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN locale 

RUN apt install -y software-properties-common
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

RUN apt install -y \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures

RUN mkdir -p /ros2_ws/src && \
cd /ros2_ws && \
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

RUN apt upgrade -y

RUN cd /ros2_ws && rosdep init && \
rosdep update && \
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"


RUN apt install -y gfortran




RUN cd /ros2_ws/ && colcon build --symlink-install 

RUN cd /ros2_ws/ && . /ros2_ws/install/local_setup.bash


WORKDIR /ros2_ws
#COPY ./entrypoint_talker.sh /ros2_ws/entrypoint.sh

RUN mkdir /var/lib/theconstruct.rrl
#COPY ./cyclonedds.xml /var/lib/theconstruct.rrl/

#ENTRYPOINT ["/bin/bash", "-c", "/ros2_ws/entrypoint.sh"]
CMD ["/bin/bash"]