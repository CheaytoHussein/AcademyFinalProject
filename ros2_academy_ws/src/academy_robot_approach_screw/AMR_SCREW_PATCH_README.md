# AMR Screw Task — Patch

This patch adds:
- `academy_robot_approach_screw` (C++): ROS 2 **Action** server `/approach_screw` that drives the base forward using `LaserScan` and stops at a threshold distance. Feedback is the remaining distance.
- `academy_robot_pose_interfaces` (IDL): **Service** `GetScrewPose` for on-demand screw pose estimation.
- `academy_robot_pose_estimator/screw_pose_service` (C++): **Service server** `/get_screw_pose` that calls your pointcloud server, runs registration with the provided STL mesh, publishes a latched `pose_estimate`, and returns the pose in the service response.
- `academy_robot_pick_place` (C++): Optional **orchestrator** that chains the approach action, pose service, and a skeleton MoveIt2 pick&place (adjust group names).

## Build

From your workspace root:

```bash
cd ros2_academy_ws
colcon build --symlink-install --packages-select \
  academy_robot_approach_screw \  academy_robot_pose_interfaces \  academy_robot_pose_estimator \  academy_robot_pick_place
source install/setup.bash
```

> If you previously built, consider a clean build for these packages:
> `rm -rf build/ install/ log/ && colcon build --symlink-install --packages-select ...`

## Run

1) **Approach server** (drives using /scan → /cmd_vel):
```bash
ros2 launch academy_robot_approach_screw approach_screw.launch.py
```
Send a goal:
```bash
ros2 action send_goal /approach_screw academy_robot_approach_screw/action/ApproachScrew "{stop_distance: 0.35, max_forward_speed: 0.15, timeout_sec: 30.0}"
```

2) **Screw pose service** (registration with STL → PoseStamped):
```bash
ros2 launch academy_robot_pose_estimator screw_pose_service.launch.py
```
Call the service:
```bash
ros2 service call /get_screw_pose academy_robot_pose_interfaces/srv/GetScrewPose "{}"
```
You should also see a latched topic:
```bash
ros2 topic echo /pose_estimate
```

3) **Pick & Place (skeleton)**:
```bash
ros2 launch academy_robot_pick_place pick_place.launch.py
```
> Adjust MoveIt2 planning group names in `pick_place_orchestrator.cpp` (`"arm"`, `"gripper"`) to match your config.

## Parameters & Topics

- **Approach action**:
  - Params: `scan_topic` (`/scan`), `cmd_vel_topic` (`/cmd_vel`), `window_deg` (20), `stop_distance` (0.35), `max_forward_speed` (0.15), `speed_kp` (0.8)
  - Feedback: remaining distance = `min_range_ahead - stop_distance`

- **Screw pose service**:
  - Params: `mesh_path` (defaults to your repository STL), `pointcloud_service` (`/depth_to_pointcloud/get_latest`), voxel/normal/feature radii.
  - Publishes: `pose_estimate` (latched) for visualization.

## Notes

- The approach server assumes the screw is placed **directly ahead** and uses the minimum range in a small angular window to stop at a safe distance.
- The pose service reuses your registration pipeline. If needed, tune parameters in the launch file.
- The pick&place orchestrator is a starting point; grasping and group names likely need adaptation to your MoveIt2 setup.
