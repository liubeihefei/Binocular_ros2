#!/bin/bash
export ROS_LOG_DIR=/home/horsefly/下载/Binocular_ros2/logs
# source /opt/ros/<ros2-distro>/setup.bash
source /home/horsefly/下载/Binocular_ros2/install/setup.bash
ros2 launch pub pub_launch.py
