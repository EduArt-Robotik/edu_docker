# Recommended Configurations
The `slam_toolbox` node can be configured via the `slam.yaml` file that can be found in `edu_docker/slam_tool_box/launch_content`. There are some values we recommend you adjust to your robot. The values are dependent on the sensors you use, the hardware the container is running on and the overall load caused by other software.
- `resolution` - default value: `0.025 m` <br> 
Controls the size of the map-pixels (default 0.025 m). Increase this value to lower the load on your CPU.
- `max_laser_range` - default value: `12 m` <br>
The maximum range of your LIDAR. Adjust this value to fit your laser scanner.

# Load a map
1. Record a map with `slam_toolbox`
2. Call the map save service of `slam_toolbox`:
```bash
ros2 service call /eduard/orange/slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: '<name>'"
```
Enter the filename at `<name>`. This service creates a `.yaml` and a `.pgm` file. 
3. Serialize the map:
```bash
ros2 service call /eduard/orange/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: ''"
```
This service converts the previous generated map files into a `.data` and a `.posegraph` file which now can be loaded by `slam_toolbox`.
4. Localize with the map
Set the mode of `slam_toolbox` in the parameter file to `localization` instead of `mapping` and add a filename:
```yaml
    #...
    mode: localization
    #...
        map_file_name: <mapName>
    #...
```
The filename doesn't need a extension.

>Issue: Can't set initial pose (`/initialpose`) because topic is not subscribed by `slam_toolbox` despite the information in the [documentation](https://github.com/SteveMacenski/slam_toolbox/tree/ros2?tab=readme-ov-file#localization).

# Get the Pose of the Robot in the Map
`slam_toolbox` publishes the pose of the robot on a topic called `<robotnamespace>/pose`, but only every time it updates the map. Since the update interval depends on the traveled distance (configurable parameter file), this isn't really useful as an input for a control.
A workaround is to analyze the `tf` relations. This can also be automated in a ROS2 node.
1. Install `tf2-tools`:
```bash
sudo apt install ros-jazzy-tf2-tools
```
2. Get the transformation between `map` and `base_footprint`
```bash
ros2 run tf2_ros tf2_echo <robotnamespace>/map <robotnamespace>/base_footprint
```
3. Analyze the transformation:
This is how the output could look like:
```bash
At time 1741605611.617410770
- Translation: [-2.087, -0.785, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.222, 0.975]
- Rotation: in RPY (radian) [0.000, 0.000, -0.449]
- Rotation: in RPY (degree) [0.000, 0.000, -25.706]
- Matrix:
  0.901  0.434  0.000 -2.087
 -0.434  0.901 -0.000 -0.785
 -0.000  0.000  1.000  0.000
  0.000  0.000  0.000  1.000
```
The translation `[x,y,z]` describes the pose of the robot in the map frame. 



# Rviz2 Plugin
`slam_toolbox` has a Rviz2 plugin which can be used to control the mapping.

1. Open Rivz2:
```bash
ros2 run rviz2 rviz2
```
2. Choose "Panels" at the top left tabs and select "Add New Panel"
3. Select the "SlamToolboxPlugin"
The plugin should now appear on the bottom left
Alternatively you can also use the Rviz2 configuration that comes the installation. It is located here:
```bash
/opt/ros/jazzy/share/slam_toolbox/config/slam_toolbox_default.rviz
```

# Commands
You can have a look on [GitHub](https://github.com/SteveMacenski/slam_toolbox?tab=readme-ov-file#api) page of `slam_toolbox` for an overview of the available services and topics.

- Save a recorded map for `nav2`
```bash
ros2 service call /<robotnamespace>/slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
  data: 'mymap'"
```
Map gets saved at `edu_docker/slam_tool_box/launch_content` as a `.pgm` and a `.yaml` file.

- Save a recorded map for `slam_toolbox` (untested)
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: 'testmap'}"
```
