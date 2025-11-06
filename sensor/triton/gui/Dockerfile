FROM ubuntu:jammy

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
# COPY ./AccerionControlCenter_7.1.0.3.0_amd64_22_04.deb /tmp/
# COPY ./TritonV1_7.1.1-r.asu /tmp/

RUN \
    cd /tmp/ \
    && apt install ./AccerionControlCenter_7.1.1.3.0_amd64_22_04.deb -y

# Start Accerion Control Center
CMD [ "/usr/share/AccerionControlCenter/bin/AccerionControlCenter" ]
