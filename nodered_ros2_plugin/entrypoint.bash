#!/bin/bash

# Setup ROS 2  environment
source /opt/vulcanexus/${ROS_DISTRO}/setup.bash
# source /opt/ros/humble/setupt.bash
source /opt/is/setup.bash
export NODE_PATH=/usr/lib/node_modules
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec "$@"