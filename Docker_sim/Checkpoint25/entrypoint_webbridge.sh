#! /bin/bash 

source /ros1_ws/devel/setup.bash

echo "$(date +'[%Y-%m-%d %T]') Starting rosbridge server..."
roslaunch course_web_dev_ros --wait web2.launch &
echo "$(date +'[%Y-%m-%d %T]') Starting tf2_web server..." 
roslaunch course_web_dev_ros --wait tf2_web.launch  &
echo "$(date +'[%Y-%m-%d %T]') Starting action server..." 
#rosrun course_web_dev_ros tortoisebot_action_server.py &


rosrun rqt_image_view rqt_image_view /camera/image_raw
