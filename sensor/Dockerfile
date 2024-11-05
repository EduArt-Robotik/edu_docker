FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

ENV ROS_DISTRO jazzy
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# install laser filters
USER root
RUN \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
      ros-$ROS_DISTRO-laser-filters \
      libcurl4-openssl-dev \
      libjsoncpp-dev \
      ros-$ROS_DISTRO-sick-scan-xd

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./rplidar_ros /home/$USER/ros/src/rplidar_ros
COPY ./sllidar_ros2 /home/$USER/ros/src/sllidar_ros2
COPY ./sick_lidar_localization /home/$USER/ros/src/sick_lidar_localization
USER root
RUN cp -f /home/$USER/ros/src/sick_lidar_localization/package_ros2.xml /home/$USER/ros/src/sick_lidar_localization/package.xml
USER $USER

WORKDIR /home/$USER/ros

# build rplidar package
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DROS_VERSION=2 -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]
