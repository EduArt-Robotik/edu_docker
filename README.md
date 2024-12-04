# edu_docker

Provides Dockerfiles for all EduArt's robots including deployment and launch scripts.

## ROS2 Middleware

|<img src='warning.png' height='100' width='400'/>   |    Version 1.3.x of edu_docker switches its ROS2 middleware from FarstRTPS (the ROS2 default) to Cyclone DDS! This change is important because these two middlewares are not fully compatible. Therefore, if you update to version 0.5.x you have to make your sure that your whole ROS2 infrastructure uses the same middleware!    <br> <br>    If you wish to keep FastRTPS as your middleware, you can do this by simply editing a parameter file. Please refer to [this guide](https://github.com/EduArt-Robotik/edu_robot/blob/main/documentation/update/changing-middleware.md) on how to adjust the middleware settings for the docker containers and your system! |
|---|----|

## Sensors

Please see [here](sensor/README.md) for details.