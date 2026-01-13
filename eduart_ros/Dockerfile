FROM ubuntu:noble

# using bash
SHELL ["/bin/bash", "-c"]
ENV UBUNTU_DISTRO=noble
ENV ROS_DISTRO=jazzy
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

USER root
RUN echo "installing ros jazzy" \
    && apt-get update \
    && apt-get -y upgrade \
    && apt-get install -q -y --no-install-recommends \
        apt-utils \
        software-properties-common \
        nano \
        curl \
        gnupg \
        git \
        build-essential \
        sudo \
        iproute2 \
        net-tools \
        wget \
    && add-apt-repository universe \
    && apt-get update \
########################################################
# Install ROS
## Get package archive key and ROS source list entry.
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && sudo dpkg -i /tmp/ros2-apt-source.deb \
    && apt-get update \
    && apt-get install apt-utils \
########################################################
# Install ROS packages
    && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    software-properties-common \
    ros-dev-tools \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-rmw-zenoh-cpp \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-diagnostic-aggregator

# Upgrade System
RUN \
    apt-get update \
    && apt-get -y upgrade

# write source commands to .bashrc -> no need to source afterwards
RUN \
########################################################
# Add non root user for development.
    useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
########################################################
# Add exports and sources to bash rc for easy handling.
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/ros/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

# work with user user
USER $USER

# set up iotbot workspace
RUN mkdir -p /home/$USER/ros/src
