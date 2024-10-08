FROM eduartrobotik/eduart-ros-base:humble-1.2.2

ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

# install laser filters
USER root
RUN \
    apt-get update \
    && apt-get install -q -y --no-install-recommends \
    ros-$ROS_DISTRO-laser-filters


USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./rplidar_ros /home/$USER/ros/src/rplidar_ros
COPY ./sllidar_ros2 /home/$USER/ros/src/sllidar_ros2
COPY ./iot_cyclone_dds_profile.xml /home/$USER/
ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml
WORKDIR /home/$USER/ros

# build rplidar package
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# launch rplidar node
CMD echo "launching rplidar control software" \
    && ros2 launch rplidar_ros rplidar.launch.py
