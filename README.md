# ROS2 Robot Tank

## Server

Install docker

Run `./docker-build.server.sh` to build the image.

Run `./docker-run.server.sh` to run the joystick controller node.

Instructions for getting xbox one controller working over bluetooth. Note disabling ERTM was not required on Ubuntu 22.04: https://www.addictivetips.com/ubuntu-linux-tips/xbox-one-controllers-over-bluetooth-linux/


## Raspberry Pi 5 - Ubuntu 24.04

### Install librealsense

```
sudo apt update && sudo apt install ros-${ROS_DISTRO}-librealsense2* ros-${ROS_DISTRO}-realsense2-*
```

### Install gpiod dependencies

```
sudo apt update && sudo apt install libgpiod-dev gpiod python3-libgpiod
```

You also might need to update some udev rules to allow your user access to gpio. See https://github.com/warthog618/go-gpiocdev/issues/10#issuecomment-1563028537


## Run the ros2 node

```
source ./install/setup.bash
ros2 launch rpi rpi.launch.py
```

### Docker Usage 
Install docker and clone the repo onto a raspberry pi,

Run `./docker-build.rpi.sh` to build the image.

Run `./docker-run.rpi.sh` to run the motor controller node.


## Pin Mappings
The raspberry pi pin mappings are:

Left Motor Step: 17
Left Motor DIR:  27
Left Motor EN:   22
Right Motor Step: 23
Right Motor DIR:  24
Right Motor EN:   25

