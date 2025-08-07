# Depth to PointCloud Converter

A simple ROS2 Python node that subscribes to depth images, converts them to point clouds using Open3D, and publishes the raw point clouds.

## Quick Start

### Launch the converter node
```bash
# Build and source
colcon build --packages-select depth_to_pointcloud
source install/setup.bash

# Launch depth to pointcloud converter
ros2 launch depth_to_pointcloud depth_to_pointcloud.launch.py
```

## Dependencies

Install Python dependencies using the provided requirements file:

```bash
pip install -r requirements.txt
```

## What it does

This node:
1. Subscribes to depth images from `/front_rgbd_camera/depth/image_raw`
2. Subscribes to camera info from `/front_rgbd_camera/depth/camera_info`
3. Converts depth images to 3D point clouds using Open3D
4. Publishes raw point clouds to `/point_cloud`

## RViz Visualization

The point cloud is published to `/point_cloud` and can be visualized in RViz using a PointCloud2 display.

## For Students

This node provides a clean starting point for point cloud processing projects. Students can create their own nodes that:
- Subscribe to `/point_cloud` 
- Implement filtering, clustering, and object detection algorithms
- Publish detected object poses and processed point clouds