FROM eduartrobotik/eduart-ros-base:humble-1.0.0

ENV ROS_DISTRO humble
ENV USER user
ENV DEBIAN_FRONTEND=noninteractive

USER $USER

# set up ros workspace
COPY ./run.sh /home/$USER/run.sh
COPY ./ipc_cyclone_dds_profile.xml /home/$USER/

ENV CYCLONEDDS_URI=/home/$USER/ipc_cyclone_dds_profile.xml

# launch rplidar node
CMD /home/user/run.sh
