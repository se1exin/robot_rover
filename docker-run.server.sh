# Make sure we can access X11 from docker
xhost +local:docker

# Docker seems to be having issues discovering other ROS2 nodes, so we manually specify the robot IP instead
ROBOT_IP_ADDRESS=10.1.1.16

docker run --rm -it \
  --privileged \
  --network=host \
  -e ROS_STATIC_PEERS=$ROBOT_IP_ADDRESS \
  --gpus all \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  robot-tank-server \
  ros2 run rqt_image_view rqt_image_view
  
  #ros2 launch joystick_control remote_robot_control.launch.py
