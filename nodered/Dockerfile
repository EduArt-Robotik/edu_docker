FROM eduartrobotik/eduart-robot:0.3.1

ENV USER user
ENV ROS_DISTRO humble
WORKDIR /home/$USER
USER $USER

# Installing Node Red with ROS2 Plugin
RUN echo "Installing Node Red with ROS2 Plugin" \
    cd /home/$USER \
    && wget https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh \
    && chmod u+x install.sh \
    && ./install.sh \
    && export NVM_DIR="$HOME/.nvm" \
    && [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion" \
    && nvm install 18 node \
    && npm install -g --unsafe-perm node-red \
    && npm install -g node-red-ros2-plugin

# Installing eProsima Integration Service
## Installing Dependencies
USER root
RUN echo "Installing Dependencies for Integration Service" \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
    libasio-dev \
    libboost-dev \
    libboost-program-options-dev \
    libboost-system-dev \
    libcurl4-openssl-dev \
    libcurlpp-dev \
    libssl-dev \
    libwebsocketpp-dev \
    libyaml-cpp-dev \
    ros-$ROS_DISTRO-rmw-fastrtps-cpp \
    ros-$ROS_DISTRO-fastrtps \
    && apt-get clean

## Installing Services
# USER $USER
RUN echo "Installing eProsima Integration Service" \
    && cd /home/$USER/ros/src \
    && git clone https://github.com/eProsima/Integration-Service.git is \
    && sed -e "s/-Werror //g" -i is/utils/conversion/json-xtypes/CMakeLists.txt \
    && git clone https://github.com/eProsima/WebSocket-SH.git \
    && git clone https://github.com/eProsima/ROS2-SH.git \
    && git clone https://github.com/eProsima/FIWARE-SH.git \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    && MAKEFLAGS="-j10" colcon build --cmake-args -DIS_ROS2_SH_MODE=DYNAMIC --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/is

COPY launch-nodered.sh /home/$USER/

CMD ["bash", "launch-nodered.sh"]
