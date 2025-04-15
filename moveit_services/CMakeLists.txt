cmake_minimum_required(VERSION 3.8)
project(moveit_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
# find_package(moveit_visual_tools REQUIRED)
find_package(cp25_custom_interfaces REQUIRED) 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(planning_sim_scene_service src/planning_sim_scene_service.cpp)
ament_target_dependencies(planning_sim_scene_service rclcpp std_srvs moveit_msgs )

add_executable(planning_realrobot_scene_service src/planning_realrobot_scene_service.cpp)
ament_target_dependencies(planning_realrobot_scene_service rclcpp std_srvs moveit_msgs )

add_executable(moveit_sim_service src/moveit_sim_service.cpp)
ament_target_dependencies(moveit_sim_service rclcpp std_srvs moveit_msgs moveit_core
	moveit_ros_planning_interface moveit_ros_planning )

add_executable(moveit_realrobot_service src/moveit_realrobot_service.cpp)
ament_target_dependencies(moveit_realrobot_service rclcpp std_srvs moveit_msgs moveit_core
	moveit_ros_planning_interface moveit_ros_planning )

add_executable(moveit_goto_pose_topic_service src/moveit_goto_pose_topic_service.cpp)
ament_target_dependencies(moveit_goto_pose_topic_service rclcpp moveit_msgs moveit_core 
cp25_custom_interfaces moveit_ros_planning_interface moveit_ros_planning)

add_executable(moveit_goto_pose_topic_server_service_client src/moveit_goto_pose_topic_server_service_client.cpp)
ament_target_dependencies(moveit_goto_pose_topic_server_service_client rclcpp geometry_msgs cp25_custom_interfaces)



add_executable(goal_pose_trajectory
    src/goal_pose_trajectory.cpp)
target_include_directories(goal_pose_trajectory
    PUBLIC include)
ament_target_dependencies(goal_pose_trajectory
    ament_cmake
    rclcpp
    rclcpp_action
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_planning
    control_msgs)


install(TARGETS
   planning_sim_scene_service
   planning_realrobot_scene_service
   moveit_sim_service
   moveit_realrobot_service 
   moveit_goto_pose_topic_service
   #goal_pose_trajectory # test only
   moveit_goto_pose_topic_server_service_client
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
