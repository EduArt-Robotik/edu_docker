FROM eduart-vnc-ros:0.1.0

# using bash
SHELL ["/bin/bash", "-c"]
ENV UBUNTU_DISTRO=jammy
ENV ROS_DISTRO=jazzy
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

RUN \
    apt update \
    && apt install qt6-base-dev -y

COPY ./AccerionControlCenter_7.1.1.3.0_amd64_22_04.deb /tmp/

RUN \
    cd /tmp/ \
    && apt install ./AccerionControlCenter_7.1.1.3.0_amd64_22_04.deb -y

# Alternatively, to run Accerion Control Center via supervisord
# RUN echo "[program:accerion_control_center] \n\
# command=/usr/share/AccerionControlCenter/bin/AccerionControlCenter \n\
# autostart=true \n\
# autorestart=true \n\ " >> /home/user/supervisor/supervisord.conf

# Start Accerion Control Center
CMD [ "/usr/share/AccerionControlCenter/bin/AccerionControlCenter" ]
