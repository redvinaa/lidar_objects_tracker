# Lidar Objects Tracker

This package provides a ROS 2 node for tracking objects in LiDAR scans. It works by first clustering the scan data and then applying a Linear Multi-Bernoulli (LMB) tracking algorithm. Each track is maintained using a Kalman filter. The tracked objects are published and visualized in RViz as a `MarkerArray`.

![Demo](/demo.gif)

## Usage

Build the package and run the node:

```bash
ros2 run lidar_objects_tracker objects_tracker_node
```

Alternatively, you can use the launch file:

```bash
ros2 launch lidar_objects_tracker lidar_objects_tracker_launch.py
```

## TODOs

* Cluster (measurement) confidence (human size, shape, etc.)
* Tests for each component
* Optionally publish static (and dynamic?) scans
* Improve clustering
* Delete leftover bounding box markers
