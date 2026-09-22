# start from eduart-robot-control base image, which includes eduart packages
FROM eduartrobotik/eduart-robot-control:0.5.0

ENV ROS_DISTRO=jazzy
ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install desktop and vnc server
USER root
RUN \
    apt update \
    && apt install -y \
      xfce4 \
      xfce4-terminal \
      x11vnc \
      xvfb \
      novnc \
      websockify \
      supervisor \
      dbus-x11

# Using own background image for XFCE by replacing the default background image.
COPY eduart-wallpaper.svg /usr/share/backgrounds/xfce/xfce-shapes.svg

# VNC setup
USER $USER
RUN mkdir -p /home/user/supervisor/logs /home/user/supervisor/run
COPY --chown=user:user supervisord.conf /home/user/supervisor/supervisord.conf
RUN chmod a+w /home/user/supervisor/supervisord.conf
