# Depth PCD Processor

A ROS2 Python node that subscribes to depth images, converts them to point clouds using Open3D, and publishes the processed point clouds.

## Quick Start

### Package-Only Launch (PCD processor only)
```bash
# Build and source
colcon build --packages-select depth_pcd_processor
source install/setup.bash

# Launch PCD processor with RViz
ros2 launch depth_pcd_processor pcd_processor.launch.py
```

### Full Simulation Launch (World + Robot + PCD processor)
```bash
# Launch complete simulation with PCD processing
ros2 launch depth_pcd_processor complete_simulation.launch.py
```

## Debug Mode

Enable debug mode to see detailed logs about:
- Camera intrinsics and image dimensions
- Point cloud coordinate ranges
- Depth filtering statistics
- TF transformation details

```bash
# Add debug_mode:=true to any launch command
ros2 launch depth_pcd_processor pcd_processor.launch.py debug_mode:=true
```

or

```bash
# Launch complete simulation with PCD processing
ros2 launch depth_pcd_processor complete_simulation.launch.py debug_mode:=true
```

## Dependencies

Install Python dependencies using the provided requirements file:

```bash
pip install -r requirements.txt
```

## RViz Visualization

The processed point cloud is published to `/processed_point_cloud` and can be visualized in RViz using a PointCloud2 display. 