# ROS2 Robot Tank

## Server

Clone `twist_stamper` to the root of this project:

```
git clone https://github.com/joshnewans/twist_stamper.git
```

### Simulation via Gazebo

Launch Gazebo
```
ros2 launch robot_tank_desc gazebo.launch.py
```

Launch RViz
```
rviz2 -d src/robot_tank_desc/config/view_robot.rviz
```


### Physical Robot

Launch Robot Server
```
ros2 launch robot_tank_desc robot.launch.py
```

Launch RViz
```
rviz2 -d src/robot_tank_desc/config/view_robot.rviz
```


### SLAM/Nav2

```
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/robot_tank_desc/config/mapper_params_online_async.yaml use_sim_time:=true
```

```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

Launch RViz
```
rviz2 -d src/robot_tank_desc/config/robot_with_map.rviz
```

### Docker
Install docker

Run `./scripts/server.docker-build.sh` to build the image.

Run `./scripts/server.docker-run.sh` to run the joystick controller node.

Instructions for getting xbox one controller working over bluetooth. Note disabling ERTM was not required on Ubuntu 22.04: https://www.addictivetips.com/ubuntu-linux-tips/xbox-one-controllers-over-bluetooth-linux/

## Raspberry Pi 5 - Ubuntu 24.04

### Install gpiod dependencies

```
sudo apt update && sudo apt install libgpiod-dev gpiod python3-libgpiod
```

You also might need to update some udev rules to allow your user access to gpio. See https://github.com/warthog618/go-gpiocdev/issues/10#issuecomment-1563028537

### Enable Serial

Edit /boot/firmware/config.txt and add the following lines:

```
enable_uart=1
dtoverlay=disable-bt
```

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

Left Motor Step: 27
Left Motor DIR:  17
Left Motor EN:   22
Right Motor Step: 6
Right Motor DIR:  6
Right Motor EN:   13

Lidar TX: 15 (RPi UART0 RX)
Lidar RX: 14 (RPi UART0 TX)

