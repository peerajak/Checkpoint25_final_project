## Example 2: run the bridge and exchange images
https://github.com/ros2/ros1_bridge

docker build -t test:ros1_bridge -f ../../dockerRos2Talker_dockerRos1Bridge_dockerRos1Listener/Dockerfile_Ros1Bridge .
docker build -t test:ros1_usbcam_show  -f ../docker_ros1USB_ros1_show/Dockerfile .
docker build -t test:ros2_usbcam_show  -f ../docker_ros2USB_ros2_show/Dockerfile .
docker build -t ros1_and_web_bridge:v1 -f ../Checkpoint25/Dockerfile_Ros1_Web_Bridge .


### ros2 hold camera, and ros1 show 

docker compose -f docker-compose_r2cam.yml up


