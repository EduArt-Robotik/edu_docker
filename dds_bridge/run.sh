#!/bin/bash
source /opt/ros/humble/setup.bash && ros2 daemon start && ros2 topic pub /dds_bridge std_msgs/msg/String "{data: 'sent from dds bridge'}"

while true
do
  sleep 1000
done
