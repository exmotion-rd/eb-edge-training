#!/bin/bash
ROS_DISTRO=humble
if [ $# -eq 1 ]; then
    ROS_DISTRO=$1
fi
docker build -t ros2-${ROS_DISTRO}-env --build-arg ROS_DISTRO=${ROS_DISTRO} .
