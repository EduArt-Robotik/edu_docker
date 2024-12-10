# edu_docker

Provides Dockerfiles for all EduArt's robots including deployment and launch scripts.

## ROS2 Middleware

Since version 0.5.0 it is possible to select the ROS2 middleware if edu_robot is used in a Docker container. If you want to configure it, you can do so by following [these instructions](https://github.com/EduArt-Robotik/edu_robot/blob/main/documentation/update/changing-middleware.md).

Since December 2024, EduArt robots are delivered with Cyclone DDS as middleware by default. If you would like to use FastRTPS, please also refer to [these instructions](https://github.com/EduArt-Robotik/edu_robot/blob/main/documentation/update/changing-middleware.md).

## Sensors

Please see [here](sensor/README.md) for details.
