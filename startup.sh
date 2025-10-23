#!/bin/bash
export ROS_LOG_DIR=/home/horsefly/下载/Binocular_ros2/logs
# source /opt/ros/<ros2-distro>/setup.bash
source /home/horsefly/下载/Binocular_ros2/install/setup.bash
chmod 666 /dev/video0
ros2 run pub pub
