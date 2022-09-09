ARG ROS_DISTRO=foxy
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV DEBIAN_FRONTEND noninteractive
ENV DEBCONF_NOWARNINGS yes
ENV APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE yes
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 依存パッケージとROS2のバイナリをインストール
RUN apt-get update \
 && apt-get install --no-install-recommends -y \
    wget \
    curl \
    dirmngr \
    gnupg2 \
    lsb-release \
    python3-rosdep2 \
    python3-colcon-ros \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro

# ローカルでビルドするパッケージの取得
ARG ROS_WS=/usr/src/ros2/${ROS_DISTRO}
WORKDIR ${ROS_WS}/src
RUN apt-get update \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && git clone https://github.com/ros-simulation/gazebo_ros2_control -b ${ROS_DISTRO} \
 && rosdep fix-permissions && rosdep update \
 && rosdep install -r --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -y

# 取得したパッケージのビルド
WORKDIR ${ROS_WS}
RUN  apt-get update \
 && . /opt/ros/${ROS_DISTRO}/setup.sh \
 && colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release \
 && rm -rf /var/lib/apt/lists/*

# ROS2環境の設定ファイルの用意
RUN sed --in-place -e 's/set -e//' /ros_entrypoint.sh \
 && sed --in-place --expression \
        '$isource "'"${ROS_WS}/install/local_setup.bash"'"' /ros_entrypoint.sh \
 && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
 && echo "source ${ROS_WS}/install/local_setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
