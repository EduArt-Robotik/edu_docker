#!/bin/bash
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source /home/user/ros/install/setup.bash
source /opt/is/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh" [ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"
node-red
