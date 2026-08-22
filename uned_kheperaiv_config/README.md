# uned_kheperaiv_config

`ament_cmake` package with the environment configuration shared by the rest of this repo: 3D models, the unified launch file, and RViz/RQT resources.

## Structure

- **`urdf/`**, **`meshes/`**, **`src/replace.py`**: the Khepera IV's URDF description — merged in from the former standalone `uned_khepera_description` package (it wasn't worth keeping separate, same call as the `uned_crazyflie_common` → `uned_crazyflie_driver` absorption in the Crazyflie repo). `urdf/khepera.xml` is the source template (`${suffix}`/`${topic_ns}` placeholders); the build generates `kheperaXX.urdf` for X in 01-40 via `src/replace.py`, installed alongside the 3 source URDF files. `khepera.xml` carries the Gazebo Classic plugins (`libgazebo_ros_diff_drive.so`, `libgazebo_ros_p3d.so`) used by `uned_kheperaiv_gazebo`.
- **`model/`**: an alternative, Gazebo-native SDF model of the Khepera IV (`model.sdf`/`model.config`, meant for Gazebo's model-database spawn flow via `spawn_entity.py -database`) — see `uned_kheperaiv_gazebo/README.md` for which of the two model approaches (this one vs. the URDF above) is the one actually kept working.
- **`launch/experience.launch.py`**: the single parametrized launch file — one `ros2 launch uned_kheperaiv_config experience.launch.py config_file:=<experience>.yaml` instead of one `.launch.py` per demo. Reads `Operation`/`Experience`/`Architecture`/`CPU_Monitoring`/`Interface`/`Data_Logging`/`Robots`/`Supervisor`/`Other` sections from a `.yaml` in `resources/`. See `resources/Demo_teleop_webots.yaml` for a fully worked example.
- **`launch/urdf_visualize.launch.py`**: standalone URDF viewer in RViz (from the former `uned_khepera_description`).
- **`resources/`**: one `.yaml` per experience (see the table in the root README), plus `vicon_config.yaml`.
- **`rviz/`**, **`rqt/`**: RViz configs and RQT perspectives tied to specific demos.
- **`worlds/`**: Webots (`.wbt`) and Gazebo (`.world`) simulation worlds.

## Dependencies from other lab repositories

`uned_kheperaiv_webots` needs `multi_agent_pkg` (from `RoboticPark`) built in the same workspace — see the root README's Installation section.
