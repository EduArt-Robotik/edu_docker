FROM eduartrobotik/eduart-ros-base:jazzy-1.2.1

ENV ROS_DISTRO=jazzy
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install laser filters
USER root
RUN \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
      ros-$ROS_DISTRO-laser-filters \
      libcurl4-openssl-dev \
      libjsoncpp-dev \
      ros-$ROS_DISTRO-sick-scan-xd \
      ros-$ROS_DISTRO-angles \
      libboost-thread-dev

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
RUN mkdir -p /home/$USER/driver/

# rplidar
RUN cd /home/$USER/ros/src/ \
    && git clone --branch ros2 https://github.com/Slamtec/rplidar_ros.git \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# sllidar
RUN cd /home/$USER/ros/src/ \
    && git clone --branch main https://github.com/Slamtec/sllidar_ros2.git \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# sick localization
RUN cd /home/$USER/ros/src/ \
    && git clone --branch v6.0.0 https://github.com/SICKAG/sick_lidar_localization.git \
    && cp -f sick_lidar_localization/sick_lidar_localization_driver/package_ros2.xml sick_lidar_localization/sick_lidar_localization_driver/package.xml \
    && cp -f sick_lidar_localization/sick_lidar_localization_msgs/package_ros2.xml sick_lidar_localization/sick_lidar_localization_msgs/package.xml \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DROS_VERSION=2 -DCMAKE_BUILD_TYPE=Release

# leuze lidar
RUN cd /home/$USER/ros/src/ \
    && git clone https://github.com/thesensorpeople/leuze_rsl_ros2_drivers.git \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# ydlidar sdk for t-mini pro
USER root
RUN cd /tmp \
    && git clone --branch V1.2.7 https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK/ \
    && mkdir build \
    && cd build/ \
    && cmake .. \
    && make -j$(($(nproc)-1)) \
    && make install

USER $USER
RUN cd /home/$USER/ros/src/ \
    && git clone --branch humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# accerion driver
USER root
RUN apt-get update \
    && apt-get install -y \
        libmagicenum-dev \
        ros-$ROS_DISTRO-pcl-conversions

USER $USER
RUN cd /home/$USER/ros/src/ \
    && git clone https://gitlab.com/accerion/accerion-ros2.git \ 
    && cd accerion-ros2 \
    && git submodule update --init --recursive \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd /home/$USER/ros \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release


WORKDIR /home/$USER/ros

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
