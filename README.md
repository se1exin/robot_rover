# ROS2 Robot Tank

## Server

Install docker

Run `./docker-build.server.sh` to build the image.

Run `./docker-run.server.sh` to run the joystick controller node.


## Raspberry Pi

Install librealsense

sudo apt install ros-${ROS_DISTRO}-librealsense2*
sudo apt install ros-${ROS_DISTRO}-realsense2-*

Install docker and clone the repo onto a raspberry pi,

Run `./docker-build.rpi.sh` to build the image.

Run `./docker-run.rpi.sh` to run the motor controller node.

The raspberry pi pin mappings are:

Left Motor Step: 17
Left Motor DIR:  27
Left Motor EN:   22
Right Motor Step: 23
Right Motor DIR:  24
Right Motor EN:   25

