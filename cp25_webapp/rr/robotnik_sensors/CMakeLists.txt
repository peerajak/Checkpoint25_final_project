cmake_minimum_required(VERSION 2.8.3)
project(robotnik_sensors)

find_package(catkin REQUIRED COMPONENTS
    #xacro
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES course_web_dev_ros
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
