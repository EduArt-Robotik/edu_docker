## Install UDEV Rules

First, the udev rules must be installed. This is required for access to the Tmini-pro device. Navigate into the `tminipro` folder and install rules by:

```bash
cd ~/edu_docker/sensor/tminipro
sudo make install-udev-rules
```

Now plugin the RPLidar device. After the device is plugged it will be listed in device. Please check by:

```bash
ls /dev
```

There **tminipro** should be listed.