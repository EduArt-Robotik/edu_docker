# RP Lidar

Here is a description of how various RPLidar models are operated on our EduArt robots. A Docker image is provided for this purpose, with which all RPLidar models can be controlled.

The A2M8 is controlled by default. If a different model is to be used, this must be adapted in the launch file. Please see below how.

## Clone Repository

All skirpts used here are in this repository. To use them, please clone them with the following command:

```bash
cd ~
git clone https://github.com/EduArt-Robotik/edu_docker.git
```

## Install UDEV Rules

First, the udev rules must be installed. This is required for access to the RPLidar device. Navigate into the rplidar folder and install rules by:

```bash
cd ~/edu_docker/sensor/rplidar
sudo make install-udev-rules
```

Now plugin the RPLidar device. After the device is plugged it will be listed in device. Please check by:

```bash
ls /dev
```

There **rplidar** should be listed.

## Adapting Launch File

The provided launch file is configured to use an RPLidar A2M8. If you want to use a different model please adapt it following. If you using the A2M8 model this section can be skipped.

In the folder **launch_content** the launch file **rplidar.lauch.py** is located. Navigate to and open it by:

```bash
cd ~/edu_docker/sensor/rplidar/launch_content
nano rplidar.launch.py
```

The **IncludeLaunchDescription** has to be modified or more specific one argument has to be adapted. The launch description looks like:

```python
rplidar_node = IncludeLaunchDescription(
  PythonLaunchDescriptionSource([
    PathJoinSubstitution([
      FindPackageShare('rplidar_ros'),
      'launch',
      'rplidar_a2m8_launch.py'
    ])
  ]),
  launch_arguments={
    'serial_port' : '/dev/rplidar',
    # 'serial_baudrate' : '115200'
  }
)
```

The name of the supplied launch file must be adapted to the model to be used. The following start files are available. Please select the file that matches your Lidar model:

| Model | Launch File |
|-------|-------------|
| A1    |rplidar_a1_launch.py |
| A2M12 |rplidar_a2m12_launch.py |
| A2M7  |rplidar_a2m7_launch.py |
| A2M8  |rplidar_a2m8_launch.py |
| A3    |rplidar_a3_launch.py |
| C1    |rplidar_c1_launch.py |
| S1    |rplidar_s1_launch.py |
| S2    |rplidar_s2_launch.py |
| S2E   |rplidar_s2e_launch.py |
| S3    |rplidar_s3_launch.py |
| T1    |rplidar_t1_launch.py |

Please be aware of the used baudrate (switch on the side of the connection box). This has to fit to the used Lidar model.

## Launching Driver Using Docker

After the launch file was adapted (or not) the software could be launched using following command:

```bash
cd ~/edu_docker/sensor/rplidar
docker compose up
```

## Updating Docker Image

The docker image can updated by following commands:

>**Note**: your adapted launch file will be kept. If your running in same problems with git please contact the EduArt support.

```bash
cd ~/edu_docker/sensor/rplidar
git fetch origin
git stash
git pull --ff
git stash pop
docker compose up
```
