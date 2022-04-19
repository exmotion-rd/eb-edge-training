#!/bin/bash
ROS_DISTRO=foxy
if [ $# -eq 1 ]; then
    ROS_DISTRO=$1
fi

# DockerコンテナからのX11のアクセスを許可
xhost + local:docker

# コンテナの起動
docker run  --rm --network host \
 --name foxy \
 --gpus all \
 -e NVIDIA_DRIVER_CAPABILITIES=all \
 -v /dev/shm:/dev/shm \
 -v /tmp/.X11-unix:/tmp/.X11-unix \
 -e DISPLAY \
 -e XAUTHORITY \
 -v=$(pwd):/workspace \
 -w /workspace \
 -it \
 ros2-${ROS_DISTRO}-env 

# DockerコンテナからのX11のアクセスを削除
xhost - local:docker
