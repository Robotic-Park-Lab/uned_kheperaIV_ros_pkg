# uned_vicon_gazebo

`ament_python` package with a Vicon-to-Gazebo pose bridge for the Khepera IV (legacy Gazebo path — revisit once `uned_kheperaiv_gazebo`'s Gazebo Classic 11 update is confirmed working by Francisco, see that package's README).

## Structure

- **`vicon_gazebo.py`** (entry point `vicon_gazebo`): `ViconGazebo`, a node that instantiates one `Agent` per robot id listed in its `agents` parameter (comma-separated, e.g. `khepera01, khepera02`). Each `Agent` subscribes to `<id>/ground_truth` (`nav_msgs/Odometry`, Gazebo's own ground-truth output) and republishes just the pose on `<id>/pose` (`geometry_msgs/Pose`) — i.e. it adapts Gazebo's simulated ground truth into the same kind of `Pose` topic a real Vicon bridge would publish, so downstream nodes don't need to know whether they're running against real Vicon hardware or a Gazebo simulation.
- The node also declares an unused `topic`/`String` subscription (`listener_callback`) — leftover from the `ros2 pkg create` example template, never removed. Not touched in this pass (see `AUDIT.md` on the `doc` branch).

## Usage

```
ros2 run uned_vicon_gazebo vicon_gazebo --ros-args -p agents:="khepera01, khepera02"
```

## Tests

None beyond the standard `ament_copyright`/`ament_flake8`/`ament_pep257` lint tests, which have pre-existing failures (see `AUDIT.md` on the `doc` branch, section 2 — `uned_vicon_gazebo`'s lint debt was flagged, not fixed, in this pass). `ViconGazebo`/`Agent` are real `rclpy` nodes with subscriptions/publishers wired straight from `__init__`; testing them meaningfully needs a running ROS graph (or Gazebo itself), not attempted here.
