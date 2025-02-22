docker build -t test:ros1_bridge -f ../../dockerRos2Talker_dockerRos1Bridge_dockerRos1Listener/Dockerfile_Ros1Bridge .
docker build -t test:ros1_usbcam_show  -f ../docker_ros1USB_ros1_show/Dockerfile .
docker build -t test:ros2_usbcam_show  -f ../docker_ros2USB_ros2_show/Dockerfile .
docker compose up