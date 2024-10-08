FROM eduartrobotik/eduart-ros-base:humble-1.2.2

ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER root

RUN echo "installing nav2 and slam packages" \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup

USER $USER

#git@github.com:schmiddey/wombat_nav2_pack.git

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# launch rplidar node
CMD echo "launching rplidar control software"
