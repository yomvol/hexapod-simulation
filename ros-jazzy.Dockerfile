FROM ubuntu:24.04 AS base
ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y --no-install-recommends \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y --no-install-recommends tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get -y upgrade \
    && rm -rf /var/lib/apt/lists/*

# Install common programs
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common \
    wget \
    nano \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository universe
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib/

# Environment variables for Nvidia acceleration
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Configure XDG_RUNTIME_DIR
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root

# Install ROS2 packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    python3-argcomplete \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-control-msgs \
    ros-jazzy-controller-interface \
    ros-jazzy-xacro \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-hardware-interface \
    ros-jazzy-gz-ros2-control \
    libeigen3-dev \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros_ws
COPY /*/package.xml ./src/hexapod_simulation/

RUN rosdep init || echo "rosdep already initialized" && rosdep update

RUN rosdep install -y -r --from-paths src --ignore-src \
    && rm -rf ./src/*
    # Nuke whatever was copied before (just manifests)

# Install Gazebo Harmonic
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS2 on shell entry
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

ENV DEBIAN_FRONTEND=

###########################################
#  Develop image
###########################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    python3-pip \
    python3-setuptools \
    ros-dev-tools \
    ros-jazzy-ament-* \
    && rm -rf /var/lib/apt/lists/*

# Install colcon-common-extensions
RUN pip3 install --break-system-packages -U colcon-common-extensions

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Check if "ubuntu" user exists, delete it if it does, then create the desired user
RUN if getent passwd ubuntu > /dev/null 2>&1; then \
        userdel -r ubuntu && \
        echo "Deleted existing ubuntu user"; \
    fi && \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "Created new user $USERNAME"

# Add sudo support for the non-root user
RUN apt-get update && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Added workspace sourcing if present (e.g., mounted directory)
RUN echo "if [ -f /ros_ws/install/setup.bash ]; then source /ros_ws/install/setup.bash; fi" >> /root/.bashrc

ENV DEBIAN_FRONTEND=

############################################
# Build image
############################################
FROM dev AS builder
ENV DEBIAN_FRONTEND=noninteractive

COPY . ./src/hexapod_simulation/
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"
ENV DEBIAN_FRONTEND=

############################################
# Runtime stage
############################################
FROM base AS runtime
COPY --from=builder /ros_ws/install/ /ros_ws/install/
CMD ["/bin/bash", "-c", "source /ros_ws/install/setup.bash && exec bash"]