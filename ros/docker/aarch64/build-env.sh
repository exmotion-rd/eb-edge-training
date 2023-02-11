#!/bin/bash
ROS_DISTRO=humble
if [ $# -eq 1 ]; then
    ROS_DISTRO=$1
fi
docker build --build-arg ROS_DISTRO=${ROS_DISTRO} -t ros2-${ROS_DISTRO}-aarch64-env .
