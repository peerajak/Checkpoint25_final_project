Terminal 1
```
docker build -t test:talker_listener .
docker compose -f docker-compose-talker-hostListener.yml up
```

Terminal 2

```
export ROS_DOMAIN_ID=2
ros2 run demo_nodes_cpp listener
```
