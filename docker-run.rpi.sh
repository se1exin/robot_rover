#!/bin/bash

docker stop robot-tank-rpi
docker container rm robot-tank-rpi

source $(pwd)/scripts/detect-webcams.sh

SERVER_IP_ADDRESS=10.1.1.6

docker run -it -d \
  --privileged \
  --net=host \
  -e WEBCAM_1=$WEBCAM_1 \
  -e WEBCAM_2=$WEBCAM_2 \
  -e ROS_STATIC_PEERS=$SERVER_IP_ADDRESS \
  --device /dev/gpiochip1 \
  --volume /sys/class/gpio:/sys/class/gpio \
  -v /dev:/dev/ \
  -v /run/udev/:/run/udev/ \
  -v $(pwd)/src:/app/src \
  --restart=unless-stopped \
  --name="robot-tank-rpi" \
  robot-tank-rpi \
  ros2 launch rpi rpi.launch.py
