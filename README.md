# ALFA Extensions

 This repository provides ready-to-use extensions already tested with ALFA. They can be used as a starting point for developing new ones or for testing the different ALFA setups.

## Available extensions

- **Dummy** [[Software]](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_dummy) [[Hardware]](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_dummy) - This extension is used to demonstrate how ALFA extensions are built and used. It simply subscribes a /Pointcloud2 topic and outputs the same pointcloud into /dummy_pointcloud topic.
- **Distance Filter** [[Software]](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_distance_filter) [[Hardware]](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_distance_filter) - This extension subscribes a /Pointcloud2 topic called /velodyne_points and outputs the processed point cloud in the /distance_filter topic.  The processing of this point cloud basically consists of a distance filter where the points within a predefined range (min and max) are displayed. By default these parameters are min_distance=5m and max_distance=20m. This node also publishes some important info regarding the processing of the point cloud, such as the time (ms) it takes to process (full node and just the handler) one point cloud frame.

<!-- a normal html comment  - **DIOR** [[Software]](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_dior) [[Hardware]](https://github.com/alfa-project/alfa-extensions/tree/main/hw/ext_dior) - **TODO Write a brief description of this extension**.
-->
## Run ALFA extensions

To run ALFA extensions, execute the *ros2 run* command followed by the package name and extension name. For instance, to run the ext-dummy extension run (by default the subscriber topic is /velodine_points, but you can specify a different one when starting the node):

```sh
ros2 run ext_dummy ext_dummy /<topic-name>
```

The following output should appear for a software extension:

```sh
--------------------------------------------------------
Starting ALFA node with the following settings: 
Subscriber topic: /velodyne_points
Name of the node: ext_dummy
Extension ID: 0
Pointcloud ID: 0
Hardware Driver (SIU): false
Hardware Extension: false
--------------------------------------------------------
```

or this if hardware is enable:

```sh
Hardware Extension: true
```

The node is now ready to process pointcloud data. The point clouds used by ALFA can either come from the physical sensor connected to a Ethernet interface or from a ROS2 bag file with a previous sensor capture in a real world scenario. To read it from a ROS Bag, we recommended the following datasets:

- [Lincoln MKZ Dataset by Richard Kelley](https://richardkelley.io/data) -  Sensor: Velodyne VLP-16 Puck.
- [UMA-SAR](https://www.uma.es/robotics-and-mechatronics/cms/menu/robotica-y-mecatronica/datasets/) - Sensor: Velodyne HDL-32E.
- [Semantic KITTI](http://www.semantic-kitti.org/dataset.html#download) - Sensor: Velodyne HDL-64E.
- [Zenseact](https://www.zenseact.com/) - Sensor: Velodyne VLS-128.

**Note:** The datasets are not in the ROS2 bag format, so you have to convert them first. Instructions here: <https://github.com/tomas789/kitti2bag> or <https://github.com/amslabtech/semantickitti2bag> for Semantic KITTI dataset. However this converts only for the ROS1 format. You have to convert it again for ROS2. Hit us up if you need ready to use ROS2 bags of those datasets.

ALFA supports the following two methods to play bags and visualize data.

### ALFA-Monitor

In your desktop, open a new terminal and go to your ROS2 workspace. Source the environment again:

```sh
source ./install/setup.bash 
```

And then launch the alfa-monitor:

```sh
ros2 run alfa-monitor alfa-monitor
```

To get further details on how to use the tool go to the ALFA-Monitor [repository](https://github.com/alfa-project/extensions).

### RVIZ

Go to the your bag dir and play it the following command. In this case we use the bag alread included in the alfa-monitor repository (we unzip it first):

```sh
ros2 bag play alfa-monitor/bags/alfa_example.db3 --loop
```

The terminal where the ext_dummy is running should output:

```sh
Point cloud received
```

Before using the rviz2 tool, provided by ROS2, to vizualize the point could ouputs we need to apply a window transform for the output of ALFA dummy node and the bag point cloud:

```sh
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "dummy_pointcloud" 
```

```sh
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "velodyne" 
```

```sh
ros2 run rviz2 rviz2
```

Now the RViz window should appear:

- Add a new vizualization
- Add by topic
- add both /velodyne_points (to see the original) and /dummy_pointcloud (to see the point cloud processed by the ALFA dummy node).

### Node Topics and Services

To check topics:

```sh
ros2 topic list
```

Will outut:

```sh
/ext_dummy_alive
/ext_dummy_metrics
/ext_dummy_pointcloud
```

Echo the topics to output what their echo is outputting. E.g.:

```sh
ros2 topic echo /ext_dummy_alive 
node_name: ext_dummy
node_type: extension
config_service_name: ext_dummy_settings
current_status: 0
config_tag: Configuration
default_configurations:
- config_name: min_distance
  config: 5.0
- config_name: max_distance
  config: 20.0
```

```sh
ros2 topic echo /ext_dummy_metrics 
message_tag: ''
metrics:
- metric_name: Handler processing time
  metric: 44.0
  units: ms
- metric_name: Full processing time
  metric: 60.0
  units: ms
```

To check the services provided by the ALFA dummy:

```sh
ros2 service list
```

The dummy does not provide any parameter to be changed, but the ext_distance_filter does (**min_distance** and **max_distance**). E.g., to change the min distance to 1.0, type:

```sh
ros2 param set /ext_distance_filter min_distance 1.0
```

Change these parameters to see the output changing in the RViz tool.