FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

ENV ROS_DISTRO=jazzy
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

USER root

RUN echo "installing nav2 and slam packages" \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup

USER $USER

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/

RUN cd /home/$USER/ros/src/ \
    && git clone --branch jazzy https://github.com/schmiddey/wombat_nav2_pack.git \
    && cd .. \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && source /home/$USER/.bashrc \
    && colcon build --symlink-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# launch rplidar node
CMD echo "launching rplidar control software"
