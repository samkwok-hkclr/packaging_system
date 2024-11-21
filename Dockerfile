FROM ubuntu:22.04 AS dev_img

RUN apt update
RUN DEBIAN_FRONTEND="noninteractive" apt install -y tzdata

ENV TZ=Asia/Hong_Kong
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime \
    && echo $TZ > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata 

RUN apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

RUN apt install -y curl systemd udev software-properties-common
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && apt -y upgrade
ENV ROS_DISTRO=humble
RUN apt install -y ros-${ROS_DISTRO}-ros-base

RUN apt install -y build-essential cmake git wget dos2unix \
    python3-colcon-common-extensions python3-pip python3-rosdep python3-vcstool \
    libusb-1.0-0-dev libusb-dev \
    ros-${ROS_DISTRO}-rosbridge-server \
    ros-${ROS_DISTRO}-controller-interface \ 
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-controller-manager
    
EXPOSE 8000 9090

ENV WS_NAME=packaging_system
RUN mkdir -p /${WS_NAME}/src

WORKDIR /${WS_NAME}

COPY ./src ./src

RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths /${WS_NAME}/src/ros2_canopen --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

RUN mkdir -p /logs
ENV ROS_LOG_DIR=/logs
VOLUME [ "/logs" ]

COPY --chmod=755 ./docker/entrypoint.sh /
RUN dos2unix /entrypoint.sh 
ENTRYPOINT [ "/entrypoint.sh" ]