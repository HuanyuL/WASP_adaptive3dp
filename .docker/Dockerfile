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

FROM base as kinect
#------------------------------------------------------

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.1_amd64.deb > /tmp/libk4a1.4_1.4.1_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb > /tmp/libk4a1.4-dev_1.4.1_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1/libk4abt1.1_1.1.2_amd64.deb > /tmp/libk4abt1.1_1.1.2_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4abt1.1-dev/libk4abt1.1-dev_1.1.2_amd64.deb > /tmp/libk4abt1.1-dev_1.1.2_amd64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.1_amd64.deb > /tmp/k4a-tools_1.4.1_amd64.deb
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN echo 'libk4abt1.1	libk4abt1.1/accepted-eula-hash	string	03a13b63730639eeb6626d24fd45cf25131ee8e8e0df3f1b63f552269b176e38' | debconf-set-selections
RUN dpkg -i /tmp/libk4a1.4_1.4.1_amd64.deb
RUN dpkg -i /tmp/libk4a1.4-dev_1.4.1_amd64.deb
RUN dpkg -i /tmp/libk4abt1.1_1.1.2_amd64.deb
RUN dpkg -i /tmp/libk4abt1.1-dev_1.1.2_amd64.deb
RUN apt-get update
RUN apt-get install -y libsoundio1
RUN dpkg -i /tmp/k4a-tools_1.4.1_amd64.deb
# fix k4abtConfig.cmake Error: Azure Kinect SDK Version numbers contain exactly 3 components
RUN sed -i 's/find_dependency(k4a 1.4 REQUIRED)/find_dependency(k4a 1.4.1 REQUIRED)/' /usr/lib/cmake/k4abt/k4abtConfig.cmake

RUN pip3 install --no-cache-dir --upgrade pyk4a

RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-robot-state-publisher \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


WORKDIR /dev_ws/src
RUN git clone https://github.com/microsoft/Azure_Kinect_ROS_Driver.git




FROM kinect as bash
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

