# uned_kheperaIV_ros_pkg

> 📖 To understand this repo's branches and its contribution guide, see the [`doc`](https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg/tree/doc) branch.

ROS 2 packages and configuration files for teleoperating and simulating the Khepera IV differential-drive mobile robot in ROS 2, Webots, Gazebo and Matlab. The goal is a Hardware-in-the-Loop tool that is easy to scale and maintain.

#### Structure

- **doc**. A `.tex` file going into more detail on the repository: ROS diagrams, bibliography, useful links, etc.
- **scripts**. Auxiliary files that are not part of any ROS package: the C programs (`server.c`, `prog-template.c`) that run on the Khepera IV's own onboard Linux via `libkhepera`, Arduino/ESP32 micro-ROS prototypes, and Matlab models. See [scripts/README.md](scripts/README.md).
- **[uned_kheperaiv_config](uned_kheperaiv_config/README.md)**. ROS 2 package. Environment configuration: 3D models (now including what used to be the standalone `uned_khepera_description` package), the unified `experience.launch.py` launch file, and RViz/RQT resources.
- **[uned_kheperaiv_controllers](uned_kheperaiv_controllers/README.md)**. ROS 2 package. C++ controllers for teaching: periodic PID and event-based PID position control, event triggering. Ported from the `benchmark` branch, where it had been developed but never carried over to active development.
- **[uned_kheperaiv_driver](uned_kheperaiv_driver/README.md)**. ROS 2 package. TCP client node that talks to the C program running on the Khepera IV's own onboard Linux.
- **[uned_kheperaiv_gazebo](uned_kheperaiv_gazebo/README.md)**. ROS 2 package (formerly `uned_khepera_gazebo`). Gazebo Classic 11 simulation launch/plugins for the Khepera IV.
- **[uned_kheperaiv_gui](uned_kheperaiv_gui/README.md)**. ROS 2 package. PyQt graphical interface for handling a single Khepera IV robot.
- **[uned_kheperaiv_task](uned_kheperaiv_task/README.md)**. ROS 2 package. High-level mission/task nodes: distance-based and shape-based formation control, plus a Gazebo-specific formation driver.
- **[uned_kheperaiv_webots](uned_kheperaiv_webots/README.md)**. ROS 2 package. Virtual Khepera IV driver in Webots.
- **uned_vicon_gazebo**. ROS 2 package. Vicon-to-Gazebo pose bridge (legacy path, revisit once the Gazebo update above is confirmed working).

## Installation :book:

The target is [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) on **Ubuntu 22.04**.

### Prerequisites 📋

##### ROS 2
Install ROS 2 Humble first, following the [official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

##### Webots (for `uned_kheperaiv_webots`)
```
sudo apt install ros-humble-webots-ros2-driver
```

##### Gazebo Classic (for `uned_kheperaiv_gazebo`, `uned_vicon_gazebo`)
```
sudo apt install ros-humble-gazebo-ros-pkgs
```
Gazebo Classic 11 is the simulator that pairs with ROS 2 Humble (the newer Gazebo/Ignition line pairs with ROS 2 Jazzy and later, not used in this repo).

##### `libkhepera` (for building the onboard program in `scripts/`)
`libkhepera` is K-Team SA's proprietary SDK for the Khepera platform. It is **not vendored in this repo** — no license file grants redistribution (see `AUDIT.md` on the `doc` branch). Obtain it separately (from K-Team, or your own existing copy) and see [scripts/README.md](scripts/README.md) for how to point the build at it.

##### Matlab
TO-DO — no fixed Matlab/Simulink version or toolbox list has been pinned down yet. See [scripts/README.md](scripts/README.md).

##### Dependencies from other lab repositories
Not declared with a rosdep key (no public rosdep index entry) — clone and build in the same workspace:
- **`multi_agent_pkg`**: from [Robotic-Park-Lab/RoboticPark](https://github.com/Robotic-Park-Lab/RoboticPark), required by `uned_kheperaiv_webots` for multi-agent formation math (Lagrange multipliers for sphere/cone/ellipsoid geometries).

### Building the workspace

```
mkdir -p ~/khepera_ws/src && cd ~/khepera_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git   # for multi_agent_pkg
cd ~/khepera_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Usage 🔧

### Launching an experience

Like the Crazyflie repo, there is a **single parametrized launch file**, `uned_kheperaiv_config/launch/experience.launch.py`. Each experience is a `.yaml` file in `uned_kheperaiv_config/resources/`:

```
ros2 launch uned_kheperaiv_config experience.launch.py config_file:=Demo_teleop_webots.yaml
```

| `config_file` | Description |
|---|---|
| `Demo_teleop_webots.yaml` | 1 virtual Khepera IV in Webots, teleoperated. |
| `Demo_formation_webots.yaml` | Distributed formation control in Webots. |
| `Demo_formation_central_webots.yaml` | Centralized formation control — references a `centralized_formation_controller` executable that doesn't exist yet in `uned_kheperaiv_task`; the launch itself no longer crashes on this file (a real bug, fixed — see `AUDIT.md`), but the demo won't fully run until that node is written. |

### Simulators

- **Webots**: the actively maintained path — see [uned_kheperaiv_webots/README.md](uned_kheperaiv_webots/README.md).
- **Gazebo Classic 11**: see [uned_kheperaiv_gazebo/README.md](uned_kheperaiv_gazebo/README.md) for its current status — it's being brought up to date with the current Gazebo Classic 11 (the version that pairs with ROS 2 Humble).

### Matlab controller
TO-DO

### Hardware-in-the-Loop
TO-DO: Micro-ROS

## Authors ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Related publications :paperclip:
