# docker container rm robot-tank-rpi
# --name="robot-tank-rpi" \

SERVER_IP_ADDRESS=10.1.1.6

docker run --rm -it \
  --privileged \
  --net=host \
  -e ROS_STATIC_PEERS=$SERVER_IP_ADDRESS \
  --device /dev/gpiochip1 \
  --volume /sys/class/gpio:/sys/class/gpio \
  -v /dev:/dev/ \
  -v /run/udev/:/run/udev/ \
  -v $(pwd)/src:/app/src \
  robot-tank-rpi \
  ros2 launch rpi rpi.launch.py
  #bash
