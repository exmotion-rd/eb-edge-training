#!/bin/bash
ROS_DISTRO=humble
if [ $# -eq 1 ]; then
    ROS_DISTRO=$1
fi

# DockerコンテナからのX11のアクセスを許可
UNAME_R=$(uname -r)
if [[ ! "$UNAME_R" =~ WSL2 ]]; then
    xhost + local:docker
fi

# コンテナの起動
docker run --rm --network host \
    --name foxy \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY \
    -e XAUTHORITY \
    -e TZ=Asia/Tokyo \
    -v ros2_ws:/workspace \
    -w /workspace \
    -it \
    ros2-${ROS_DISTRO}-env 

# DockerコンテナからのX11のアクセスを削除
if [[ ! "$UNAME_R" =~ WSL2 ]]; then
    xhost + local:docker
fi
