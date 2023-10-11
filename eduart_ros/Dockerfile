FROM ubuntu:jammy

# using bash
SHELL ["/bin/bash", "-c"]
ENV UBUNTU_DISTRO jammy
ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
    apt-get update \
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
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_DISTRO main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
########################################################
# Install ROS packages
    && apt-get install -q -y --no-install-recommends \
    python3-colcon-common-extensions \
    software-properties-common \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-joy-linux \
    ros-$ROS_DISTRO-diagnostic-aggregator

# write source commands to .bashrc -> no need to source afterwards
RUN \
########################################################
# Add non root user for development.
    useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
########################################################
# Add exports and sources to bash rc for easy handling.
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /home/$USER/.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/fastrtps_profile.xml" >> /home/$USER/.bashrc \
    # && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USER/.bashrc \    
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc \
    && echo "source /home/$USER/ros/install/setup.bash" >> /home/$USER/.bashrc \
########################################################
# Clean up
    && apt-get clean

# work with user user
USER $USER

# set up iotbot workspace
RUN mkdir -p /home/$USER/ros/src
