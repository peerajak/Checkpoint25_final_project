docker build -t test:ros2_usbcam_show .
#docker run -it --rm test:ros2_usbcam_show bash
#docker run --rm -it --name gimp -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --runtime=nvidia --gpus all test:ros2_usbcam_show
xhost +local:root
docker compose up

