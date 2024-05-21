# **Anand et al. - Software**

This extension implements the ground segmentation algorithm proposed by Anand et al. [1]. By default, it subscribes a Pointcloud2 topic called <b>/velodyne_points</b> topic and outputs a point cloud into <b>/ext_anand_pointcloud</b> topic. For the output of all performance metrics, it is recommended a labeled dataset. This extension was tested with the SemanticKITTI dataset.

##### [1] B. Anand, M. Senapati, V. Barsaiyan and P. Rajalakshmi, "LiDAR-INS/GNSS-Based Real-Time Ground Removal, Segmentation, and Georeferencing Framework for Smart Transportation," in IEEE Transactions on Instrumentation and Measurement, vol. 70, pp. 1-11, 2021, Art no. 8504611, doi: 10.1109/TIM.2021.3117661.

Run the Extension:
```sh
ros2 run ext_anand ext_anand /velodyne_points
```
Output:
```sh
--------------------------------------------------------
Starting ALFA node with the following settings:
Subscriber topic: /velodyne_points
Name of the node: ext_anand
Extension ID: 0
Pointcloud ID: 0
Hardware Driver (SIU): 0
Hardware Extension: 0
--------------------------------------------------------
```

Check Extension topics:

```sh
ros2 topic list
```

Output:
```sh
/ext_anand_alive
/ext_anand_metrics
/ext_anand_pointcloud
```

Dummy Extension metrics:
```sh 
No Extension metrics are available

Note: To avoid extra processing overheads, this extension outputs its metrics after processing all the point cloud sequence by pressing CTRL+C.
```

Dummy Extension services and parameters:
```sh
No Services or parameters re available
```