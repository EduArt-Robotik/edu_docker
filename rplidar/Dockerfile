FROM eduartrobotik/eduart-ros-base:humble-1.2.0

ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
COPY ./rplidar_ros2 /home/$USER/ros/src/rplidar_ros2
COPY ./iot_cyclone_dds_profile.xml /home/$USER/
ENV CYCLONEDDS_URI=/home/$USER/iot_cyclone_dds_profile.xml
WORKDIR /home/$USER/ros

# build rplidar package
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --executor sequential --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch rplidar node
CMD echo "launching rplidar control software" \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/user/ros/install/setup.bash \
    && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp \    
    && ros2 launch rplidar_ros rplidar.launch.py
