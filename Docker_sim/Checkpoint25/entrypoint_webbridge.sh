#! /bin/bash 

source /ros1_ws/devel/setup.bash

rosparam load /ros1_ws/bridge.yaml &

echo "$(date +'[%Y-%m-%d %T]') Starting rosbridge server..."
roslaunch course_web_dev_ros --wait web2.launch &
echo "$(date +'[%Y-%m-%d %T]') Starting tf2_web server..." 
roslaunch course_web_dev_ros --wait tf2_web.launch  &
echo "$(date +'[%Y-%m-%d %T]') Starting action server..." 
#rosrun course_web_dev_ros tortoisebot_action_server.py &
roslaunch ur_description --wait robot_description.launch &

rosrun rqt_image_view rqt_image_view /wrist_rgbd_depth_sensor/image_raw 
