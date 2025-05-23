cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Generate the executable

## test_trajectory
add_executable(test_trajectory
    src/test_trajectory.cpp)
target_include_directories(test_trajectory
    PUBLIC include)
ament_target_dependencies(test_trajectory
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	rviz_visual_tools)

## cartesian_trajectory
add_executable(cartesian_trajectory
    src/cartesian_trajectory.cpp)
target_include_directories(cartesian_trajectory
    PUBLIC include)
ament_target_dependencies(cartesian_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
    moveit_ros_planning
    control_msgs)

## move_sim_arm_to_show_aruco_trajectory
add_executable(move_sim_arm_to_show_aruco_trajectory
    src/move_sim_arm_to_show_aruco_trajectory.cpp)
target_include_directories(move_sim_arm_to_show_aruco_trajectory
    PUBLIC include)
ament_target_dependencies(move_sim_arm_to_show_aruco_trajectory
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	rviz_visual_tools)


## move_real_arm_to_show_aruco_trajectory
add_executable(move_real_arm_to_show_aruco_trajectory
    src/move_real_arm_to_show_aruco_trajectory.cpp)
target_include_directories(move_real_arm_to_show_aruco_trajectory
    PUBLIC include)
ament_target_dependencies(move_real_arm_to_show_aruco_trajectory
    ament_cmake
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning_interface
	interactive_markers
	moveit_ros_planning
	control_msgs
	rviz_visual_tools)


# Install the executable
install(TARGETS 
	test_trajectory
	move_sim_arm_to_show_aruco_trajectory
	move_real_arm_to_show_aruco_trajectory
	cartesian_trajectory
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
