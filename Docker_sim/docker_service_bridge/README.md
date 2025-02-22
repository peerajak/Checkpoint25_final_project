## Example 3: run the bridge for AddTwoInts service
https://github.com/ros2/ros1_bridge


docker build -t test:ros1_bridge -f ../dockerRos2Talker_dockerRos1Bridge_dockerRos1Listener/Dockerfile_Ros1Bridge .

docker compose -f docker-compose_add2int_service.yml up