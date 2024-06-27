FROM eduartrobotik/eduart-ros-base:humble-1.2.0

ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER $USER

# install SDK for YDLidar
RUN echo "installing ydlidar sdk" \
    && cd /home/$USER/ \
    && git clone https://github.com/YDLIDAR/YDLidar-SDK.git \
    && cd YDLidar-SDK \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make \
    && sudo make install

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./rplidar_ros /home/$USER/ros/src/rplidar_ros
COPY ./sllidar_ros2 /home/$USER/ros/src/sllidar_ros2
COPY ./ydlidar_ros2_driver /home/$USER/ros/src/ydlidar_ros2_driver
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
