# Wrapper script to run the control node on raspberry pi

source ./install/setup.bash

# Needed for docker communication
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run rpi_comm rpi_comm