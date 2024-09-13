ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO-ros-base as base

ARG ROS_DISTRO


ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM xterm-256color

RUN apt-get update && apt-get install -y --no-install-recommends\
    ssh \
    git \
    curl \
    terminator \
    build-essential \
    cmake \
    iputils-ping \
    python3-pip \
    python3-flake8 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN pip3 install --no-cache-dir --upgrade pip\
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    black \
    numpy \
    opencv-python \
    requests \
    -U scikit-learn \
    pykalman \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Intall ROS pkgs and dependencies
RUN apt-get update && apt-get install -y --no-install-recommends\
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-rqt-joint-trajectory-controller \   
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-rqt-gui-py \
    ros-$ROS_DISTRO-rqt-py-common \
    ros-$ROS_DISTRO-plotjuggler-ros \
    ros-$ROS_DISTRO-image-common \
    ros-$ROS_DISTRO-image-pipeline \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-plotjuggler-ros \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN mkdir -p dev_ws/src
WORKDIR /dev_ws/src

FROM base as realsense
#------------------------------------------------------

RUN apt-get update && apt-get install -y --no-install-recommends\
    software-properties-common \
    ros-$ROS_DISTRO-ddynamic-reconfigure \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-diagnostic-updater \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-key adv \
    --keyserver keyserver.ubuntu.com \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
    apt-key adv \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u \
    && apt-get install -y --install-recommends \
    librealsense2-dkms librealsense2-utils \
    librealsense2-dev librealsense2-dbg && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /dev_ws/src
RUN git clone https://github.com/rjwb1/realsense-ros.git

FROM base as bash
#------------------------------------------------------

WORKDIR /dev_ws

# Copy local pkg's from host to container
COPY . /dev_ws/src/

# Build catkin workspace
RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash &&\
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    catkin build \
    "]

# Copy and make setup script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]

# Copy and make entrypoint script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.sh / && chmod 777 /entrypoint.sh"]

ENTRYPOINT [ "/entrypoint.sh" ]

# Start bash shell
CMD ["bash"]

FROM bash as zsh
#------------------------------------------------------

# Install zsh and remove apt cache
RUN apt-get update && apt-get install -y \
    zsh \
    && rm -rf /var/lib/apt/lists/*

# Set shell to zsh
SHELL ["zsh", "chsh -s ~/.zshrc" ]

# Copy and make setup script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.zsh /dev_ws/setup.zsh && chmod 777 /dev_ws/setup.zsh"]

# Copy and make entrypoint script executable
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint_zsh.sh / && chmod 777 /entrypoint_zsh.sh"]
ENTRYPOINT ["zsh", "/entrypoint_zsh.sh" ]

# start ZSH shell
CMD ["zsh"]

