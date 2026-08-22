# uned_kheperaiv_gazebo

`ament_cmake` package with the Gazebo Classic 11 simulation launch files and support scripts for the Khepera IV (the legacy simulator path -- `uned_kheperaiv_webots` is the actively maintained one). Renamed from `uned_khepera_gazebo` for naming consistency with the rest of the repo.

> **This Gazebo Classic 11 update was implemented entirely by Claude (AI), as part of an automated repository audit/restructuring pass, and has not yet been validated by Francisco on his own setup or real Gazebo GUI session.** Everything below was verified with real, headless `gazebo`/`gzserver` runs and real topic traffic in this session's sandbox (see "What was actually verified" below) -- but that is not the same as Francisco confirming it behaves correctly for his own use on his own machine. Treat this package as **pending his review**, tracked in `AUDIT.md` on the `doc` branch.

## What was broken (found by actually trying to launch it, not just reading the code)

- `multiple_robot_Gazebo.launch.py` / `demo_formation_Gazebo.launch.py` pointed at yaml/rviz files that don't exist (`demo_teleop.yaml`, `demo_formation.yaml`, `test.rviz`), and expected a per-robot `config_path` key that the repo's current experience `.yaml` schema (`Operation`/`Robots`/...) doesn't have. `robot['pose'].split(', ')` also didn't match the actual pose format in those files (space-separated, not comma-separated).
- `simple_launch.py` uses a **different, apparently never-finished approach**: `spawn_entity.py -database khepera_IV`, which looks up a Gazebo-native SDF model by name via `GAZEBO_MODEL_PATH`. Confirmed real mismatch: `uned_kheperaiv_config/model/model.config` names the model `Khepera-IV` (capital K, hyphen), not `khepera_IV`, and `GAZEBO_MODEL_PATH` isn't set anywhere in this repo. **Left as-is, not fixed** -- it's redundant with the URDF-based path below, which is now verified working; not worth maintaining two parallel model-loading approaches. If you want this specific demo working, it needs `GAZEBO_MODEL_PATH` set to include `uned_kheperaiv_config/model/` and the model name reconciled.

## What was fixed and verified for real

`multiple_robot_Gazebo.launch.py` and `demo_formation_Gazebo.launch.py` now point at the real, existing experience files (`Demo_teleop_webots.yaml`, `Demo_formation_webots.yaml` in `uned_kheperaiv_config/resources/`), read their `Robots` dict correctly, and split `pose` on whitespace. A new `uned_kheperaiv_config/resources/khepera_gazebo_default.yaml` provides the per-robot `{task, communication}` defaults that `uned_kheperaiv_task/gazebo_driver.py` actually reads (it does `documents[robot_id]`) -- no such file existed before, so `gazebo_driver` had nothing valid to load. `task.enable` is `false` for every robot in that new file: it wires the plumbing correctly but does **not** turn on formation behavior, since the real inter-robot relationships/distances weren't specified anywhere and inventing them wasn't safe to do blind. Turning formation on for a real demo is future work, not done here.

The **URDF + `gazebo_ros` plugin path** (`uned_kheperaiv_config/urdf/khepera.xml` -> generated `kheperaXX.urdf`, `libgazebo_ros_diff_drive.so` + `libgazebo_ros_p3d.so`) turned out to already use current, working plugin syntax -- it just had nothing correctly wired to spawn it. This is the path kept and fixed; the SDF-model-database path in `simple_launch.py` is the one left as known-broken/legacy (see above).

### What was actually verified (commands run for real in this session)

```
gazebo --minimal_comms -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  <share>/uned_kheperaiv_config/worlds/UNED_RoboticParkLab_invert.world
ros2 run uned_kheperaiv_gazebo inject_entity.py <share>/uned_kheperaiv_config/urdf/khepera01.urdf 0.0 0.0 0.05 0
# -> SpawnEntity success=True, topics /khepera01/{cmd_vel,ground_truth,odom} appear
ros2 topic pub /khepera01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 20
# -> /khepera01/ground_truth position.x moved from 0.0016 to 0.661 over ~3s -- real motion, not just topic existence
```
Repeated with all 4 robots from `Demo_formation_webots.yaml` (`khepera01`-`khepera04`) spawning simultaneously with correctly namespaced topics for each.

## Structure

- `launch/multiple_robot_Gazebo.launch.py`, `launch/demo_formation_Gazebo.launch.py`: verified working (spawn + basic motion), formation task itself not turned on (see above).
- `launch/simple_launch.py`: known broken (SDF-database model lookup), not fixed, see above.
- `src/inject_entity.py`: spawns a URDF/SDF file into Gazebo via the `SpawnEntity` service. Unmodified.
- `worlds/Empty.world`: unused by the two working launch files (they use `uned_kheperaiv_config/worlds/UNED_RoboticParkLab_invert.world` instead) -- kept, not investigated further.
- `rviz/test.rviz`: **does not exist** despite being referenced by the pre-fix launch files -- the fix pointed both launch files at `uned_kheperaiv_config/rviz/default.rviz` instead, which does exist and is valid RViz YAML, but its actual display config wasn't re-verified against this package's topics.

## Tests

No functional tests added -- this package is Gazebo launch/glue code, its correctness is what the "actually verified" commands above check, not something meaningfully unit-testable without a running Gazebo instance.
