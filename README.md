# ROS2 Robot Tank

## Server

Install docker

Run `./scripts/server.docker-build.sh` to build the image.

Run `./scripts/server.docker-run.sh` to run the joystick controller node.


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

Run `./scripts/docker-build.sh` to build the image.

Run `./scripts/rpi.docker-run.sh` to run the motor controller node.


## Pin Mappings
The raspberry pi pin mappings are:

Left Motor Step: 17
Left Motor DIR:  27
Left Motor EN:   22
Right Motor Step: 23
Right Motor DIR:  24
Right Motor EN:   25

