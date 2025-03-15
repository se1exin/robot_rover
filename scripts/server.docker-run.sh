# Make sure we can access X11 from docker
xhost +local:docker

# Docker seems to be having issues discovering other ROS2 nodes, so we manually specify the robot IP instead
ROBOT_IP_ADDRESS=10.1.1.48

docker container rm robot-tank-server

docker run -it \
  --privileged \
  --network=host \
  -e ROS_STATIC_PEERS=$ROBOT_IP_ADDRESS \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v $(pwd)/src:/app/src \
  --name="robot-tank-server" \
  --gpus=all \
  robot-tank-server \
  bash
  #ros2 launch joystick_control robot_control.launch.py
  #ros2 launch joystick_control gazebo.launch.py
