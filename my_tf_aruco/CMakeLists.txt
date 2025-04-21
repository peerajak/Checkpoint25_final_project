cmake_minimum_required(VERSION 3.8)
project(my_tf_aruco)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

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



add_executable(tf2_pub_node src/tf2_pub.cpp)
ament_target_dependencies(tf2_pub_node rclcpp geometry_msgs tf2 tf2_ros)

add_executable(tf2_realrobot_pub_node src/tf2_realrobot_pub.cpp)
ament_target_dependencies(tf2_realrobot_pub_node rclcpp geometry_msgs tf2 tf2_ros)



add_executable(tf2_pub_service src/tf2_pub_service.cpp)
ament_target_dependencies(tf2_pub_service rclcpp std_srvs geometry_msgs tf2 tf2_ros)

add_executable(tf2_realrobot_pub_service src/tf2_realrobot_pub_service.cpp)
ament_target_dependencies(tf2_realrobot_pub_service rclcpp std_srvs geometry_msgs tf2 tf2_ros)


add_executable(tf2_hole_pub_service src/tf2_hole_pub_service.cpp)
ament_target_dependencies(tf2_hole_pub_service rclcpp std_srvs geometry_msgs tf2 tf2_ros)

add_executable(tf2_hole_realrobot_pub_service src/tf2_hole_realrobot_pub_service.cpp)
ament_target_dependencies(tf2_hole_realrobot_pub_service rclcpp std_srvs geometry_msgs tf2 tf2_ros)



install(TARGETS
   tf2_pub_node
   tf2_realrobot_pub_node

   tf2_pub_service
   tf2_realrobot_pub_service 
   
   tf2_hole_pub_service
   tf2_hole_realrobot_pub_service
   DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  #scripts/hole_to_camlink_baselink_tf_pub.py
  #scripts/hole_to_camlink_baselink_send_to_tf2_pub.py
  scripts/hole_to_camlink_baselink_tf_pub_service.py
  scripts/aruco_to_camlink_tf_pub.py
  scripts/aruco_to_camlink_send_to_tf2_pub.py

  scripts/hole_realrobot_to_camlink_baselink_tf_pub_service.py
  scripts/aruco_realrobot_to_camlink_tf_pub.py
  scripts/aruco_realrobot_to_camlink_send_to_tf2_pub.py
  #scripts/test_cv_bridge.py
  #scripts/test_cv_bridge_realrobot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
