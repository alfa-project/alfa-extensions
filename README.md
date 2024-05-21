# ALFA Extensions
---
 This repository provides ready-to-use ALFA extensions. They can be used as a starting point for developing or for testing the different ALFA setups.

## Available extensions

- **Dummy** [[Software]](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_dummy) - This extension is used to demonstrate how ALFA Extensions are built and used. It simply subscribes a Pointcloud2 topic called <b>/velodyne_points</b>  topic and outputs the same pointcloud into <b>/dummy_pointcloud</b> topic.
- **Distance Filter** [[Software]](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_distance_filter) - This extension subscribes a /Pointcloud2 topic called <b>/velodyne_points</b> and outputs the processed point cloud in the <b>/distance_filter</b> topic. It consists of a distance filter where only the points within a predefined range (min and max) are displayed. By default these parameters are min_distance=5m and max_distance=20m. 
- **Anand** [[Software]](https://github.com/alfa-project/alfa-extensions/tree/main/sw/ext_anand) - This extension implements the ground segmentation algorithm proposed by Anand et al. By default, it subscribes a Pointcloud2 topic called <b>/velodyne_points</b> topic and outputs a point cloud into <b>/ext_anand_pointcloud</b> topic that can be configured to print only the ground or non-ground points. For the output of all performance metrics, it is recommended a labeled dataset. This extension was tested with the SemanticKITTI dataset.

### Tested Datasets
ALFA was tested with the following datasets:

- [Lincoln MKZ Dataset by Richard Kelley](https://richardkelley.io/data) -  Sensor: Velodyne VLP-16 Puck.
- [UMA-SAR](https://www.uma.es/robotics-and-mechatronics/cms/menu/robotica-y-mecatronica/datasets/) - Sensor: Velodyne HDL-32E.
- [Semantic KITTI](http://www.semantic-kitti.org/dataset.html#download) - Sensor: Velodyne HDL-64E.
- [Zenseact](https://www.zenseact.com/) - Sensor: Velodyne VLS-128.

**Note:** The datasets are not in the ROS2 bag format, so you have to convert them first. Instructions here: <https://github.com/tomas789/kitti2bag> or <https://github.com/amslabtech/semantickitti2bag> for Semantic KITTI dataset. However this converts only for the ROS1 format. You have to convert it again for ROS2. Hit us up if you need ready to use ROS2 bags of those datasets.

For quick testing the ALFA framework with the Sequence 0 from the SemanticKITTI dataset you can use a small ros2bag available inside the [ALFA-Monitor](https://github.com/alfa-project/alfa-monitor) repository, or you can download the full bag from here:
https://drive.google.com/file/d/1OZQD10dJnyoZhAmgj--DWqrwAuQeNiHU/view?usp=sharing

## Run ALFA extensions
To run ALFA extensions, execute the <b>ros2 run</b> command followed by the package name and extension name:

```sh
ros2 run <ext_name> <ext_name> /<topic-name>
```
Each Extension provides its own output. 

### Node Topics and Services
To check available topics:

```sh
ros2 topic list
```

Will outut:
```sh
/<extension_name>_alive
/<extension_name>_metrics
/<extension_name>_pointcloud
```

Check the output of a topic:
```sh
ros2 topic echo /<extension_name>_alive 
```

To check available Extension services:

```sh
ros2 service list
```

Some Extensions provide parameters change (available in the /<extension_name>_alive topic):
```sh
ros2 param set /<extension_name> <parameter> <new_value>
```

### Point Cloud Vizualization using ALFA-Monitor
At your directory with a ros2bag dataset:
```sh
ros2 bag play <dataset_in_ros2bag format>.db3 --loop
```

The terminal where the Extension is running should output:
```sh
Point cloud received
```

Open a new terminal and go to your ROS2 workspace. Source the environment:
```sh
source ./install/setup.bash 
```

Launch the alfa-monitor:

```sh
ros2 run alfa-monitor alfa-monitor
```

To get further details on how to use the tool go to the ALFA-Monitor [repository](https://github.com/alfa-project/alfa-monitor).

### Point Cloud Vizualization using RVIZ


Before using the rviz2 tool, provided by ROS2, to vizualize the point could ouputs we need to apply a window transform for the output of ALFA dummy node and the bag point cloud:

```sh
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "<extension_topic_name>" 
```

```sh
ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "velodyne" 
```

Run RVIZ Tool
```sh
ros2 run rviz2 rviz2
```

Now in the RViz window you can:

- Add a new vizualization
- Add by topic
- add /velodyne_points to see the original point cloud
- add /<extension_topic_name> to see the Extension point cloud


