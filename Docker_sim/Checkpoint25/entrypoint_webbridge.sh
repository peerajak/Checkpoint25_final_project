#! /bin/bash 

source /ros1_ws/devel/setup.bash
roslaunch launch_cp25_ros1 ros1_tf2_webbridge.launch.xml
# rosparam load /ros1_ws/bridge.yaml 
# echo "$(date +'[%Y-%m-%d %T]') robot_description param..."
# roslaunch ur_description --wait robot_description.launch 
# echo "$(date +'[%Y-%m-%d %T]') Starting rosbridge server..."
# roslaunch course_web_dev_ros --wait web2.launch &
# echo "$(date +'[%Y-%m-%d %T]') Starting tf2_web server..." 
# roslaunch course_web_dev_ros --wait tf2_web.launch  &

# rosrun rqt_image_view rqt_image_view /wrist_rgbd_depth_sensor/image_aruco_frame
