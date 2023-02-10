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

# Check nvidia runtime
GPUS=
if [[ -n $(docker info 2>/dev/null | grep 'Runtimes.*nvidia') ]]; then
    GPUS="--gpus all"
fi

# コンテナの起動
docker run --rm --network host \
    --name ${ROS_DISTRO} \
    ${GPUS} \
    -v /dev/shm:/dev/shm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY \
    -e XAUTHORITY \
    -e TZ=Asia/Tokyo \
    -v $(pwd):/workspace \
    -w /workspace \
    -it \
    ros2-${ROS_DISTRO}-env 

# DockerコンテナからのX11のアクセスを削除
if [[ ! "$UNAME_R" =~ WSL2 ]]; then
    xhost + local:docker
fi
