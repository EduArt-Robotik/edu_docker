# Gui for Controlling Triton Accerion Sensor

## Building Docker Image

Get the deb package "AccerionControlCenter_7.1.1.3.0_amd64_22_04.deb" from the Accerion knowledge base webpage and move it to this folder. After execute following command to build the docker image.

```bash
docker build -t triton_gui .
```

## Starting Docker Container

To start the container please execute following commands:

```bash
xhost +local:docker
docker run -it --rm --env=DISPLAY --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw --net=host triton_gui
```

The container can be finished by using CTRL-C. After use the container is removed.
