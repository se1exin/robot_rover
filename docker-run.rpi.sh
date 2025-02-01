docker run -it \
  --privileged \
  --net=host \
  -v /dev:/dev/ \
  -v /run/udev/:/run/udev/ \
  robot-tank-rpi \
  ros2 run realsense2_camera realsense2_camera_node

  #   ros2 run rpi_comm rpi_comm