# Depth PCD Processor

A ROS2 Python package for processing depth images and converting them to point clouds using Open3D.

## Features

- Subscribes to depth images and camera info topics
- Converts depth images to point clouds using Open3D
- Applies depth filtering (min/max depth)
- Performs point cloud downsampling using voxel grid
- Removes statistical outliers
- Publishes processed point clouds as PointCloud2 messages

## Dependencies

- ROS2 Humble
- Python 3.8+
- Open3D
- OpenCV (cv_bridge)
- NumPy
- sensor_msgs_py

## Installation

1. Clone this package into your ROS2 workspace `src` directory
2. Install dependencies:
   ```bash
   pip install open3d opencv-python numpy
   ```
3. Build the workspace:
   ```bash
   colcon build --packages-select depth_pcd_processor
   ```
4. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### Running the Node

```bash
# Run with default parameters
ros2 run depth_pcd_processor pcd_processor_node

# Run with custom parameters
ros2 run depth_pcd_processor pcd_processor_node --ros-args \
  -p depth_topic:=/robot/front_rgbd_camera/depth/image_raw \
  -p camera_info_topic:=/robot/front_rgbd_camera/depth/camera_info \
  -p point_cloud_topic:=/processed_point_cloud \
  -p min_depth:=0.1 \
  -p max_depth:=10.0
```

### Using the Launch File

```bash
# Launch with default parameters
ros2 launch depth_pcd_processor pcd_processor.launch.py

# Launch with custom parameters
ros2 launch depth_pcd_processor pcd_processor.launch.py \
  depth_topic:=/robot/front_rgbd_camera/depth/image_raw \
  camera_info_topic:=/robot/front_rgbd_camera/depth/camera_info \
  point_cloud_topic:=/processed_point_cloud \
  min_depth:=0.1 \
  max_depth:=10.0 \
  downsample_voxel_size:=0.01 \
  remove_outliers:=true
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `depth_topic` | string | `/robot/front_rgbd_camera/depth/image_raw` | Topic for depth images |
| `camera_info_topic` | string | `/robot/front_rgbd_camera/depth/camera_info` | Topic for camera info |
| `point_cloud_topic` | string | `/processed_point_cloud` | Topic for publishing processed point clouds |
| `min_depth` | double | 0.1 | Minimum depth value in meters |
| `max_depth` | double | 10.0 | Maximum depth value in meters |
| `downsample_voxel_size` | double | 0.01 | Voxel size for downsampling (0 to disable) |
| `remove_outliers` | bool | true | Whether to remove statistical outliers |
| `outlier_nb_points` | int | 20 | Number of neighbors for outlier removal |
| `outlier_radius` | double | 0.1 | Standard deviation ratio for outlier removal |

## Topics

### Subscribed Topics

- `depth_topic` (sensor_msgs/Image): Depth images
- `camera_info_topic` (sensor_msgs/CameraInfo): Camera calibration information

### Published Topics

- `point_cloud_topic` (sensor_msgs/PointCloud2): Processed point clouds

## Processing Pipeline

1. **Depth Image Reception**: Receives depth images and camera info
2. **Camera Calibration**: Uses camera intrinsic parameters for accurate point cloud generation
3. **Point Cloud Generation**: Converts depth image to point cloud using Open3D
4. **Depth Filtering**: Removes points outside the specified depth range
5. **Downsampling**: Reduces point cloud density using voxel grid downsampling
6. **Outlier Removal**: Removes statistical outliers to clean the point cloud
7. **Publication**: Publishes the processed point cloud

## Example Usage with RViz

To visualize the processed point clouds:

1. Start the PCD processor node
2. Launch RViz:
   ```bash
   rviz2
   ```
3. Add a PointCloud2 display
4. Set the topic to `/processed_point_cloud` (or your custom topic)

## Troubleshooting

- **No camera parameters received**: Ensure the camera_info topic is being published
- **No valid points after filtering**: Adjust min_depth and max_depth parameters
- **Performance issues**: Increase downsample_voxel_size or disable outlier removal
- **Import errors**: Ensure all dependencies are installed (`pip install open3d opencv-python numpy`)

## License

TODO: Add license information 