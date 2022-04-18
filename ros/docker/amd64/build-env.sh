#!/bin/bash
#export DOCKER_BUILDKIT=0
#docker build --no-cache --network host -t ros2-foxy-env .
ROS_DISTRO=foxy
if [ $# -eq 1 ]; then
    ROS_DISTRO=$1
fi
docker build -t ros2-${ROS_DISTRO}-env --build-arg ROS_DISTRO=${ROS_DISTRO} .
