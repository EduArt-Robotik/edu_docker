FROM eduartrobotik/eduart-ros-base:humble-1.0.0

ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER $USER

# set up ros workspace
COPY ./run.sh /home/$USER/run.sh

# launch rplidar node
CMD /home/user/run.sh
