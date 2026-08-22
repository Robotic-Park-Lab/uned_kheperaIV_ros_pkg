# uned_kheperaiv_webots

`ament_python` package with the **virtual** Khepera IV driver in [Webots](https://cyberbotics.com/).

## Structure

- **`khepera_driver.py`** (`KheperaWebotsDriver`, loaded by `webots_ros2_driver` from the robot's URDF `<plugin>` tag, not via `ros2 run`): the simulated driver — motion, sensors, and formation control (distance/point/sphere/cone/ellipsoid geometries, via `multi_agent_pkg.lagrange_multipliers`).
- Shares `PIDController` and `Agent` with `uned_kheperaiv_driver` (the physical robot's driver package) — see that package's README for what was unified and what deliberately wasn't. This package used to import `Agent` (and an unused `Crazyflie_ROS2`, dead code) from `uned_crazyflie_driver`, a real dependency on the *Crazyflie* repo that this package never actually needed for anything Crazyflie-specific — removed, replaced by `uned_kheperaiv_driver`'s own `Agent`.

## Tests

No functional tests added in this pass: `KheperaWebotsDriver`'s own logic (`distance_gradient_controller`, `pose_gradient_controller`, `step()`, the geometry-tracking control loops) needs a real Webots `Robot` object to run meaningfully, which isn't available in this environment. The two classes it now imports from `uned_kheperaiv_driver` (`PIDController`, `Agent`) **are** tested — see `uned_kheperaiv_driver/README.md`'s Tests section.

**Pre-existing, not fixed here**: `khepera_driver.py`'s `copyright`/`flake8`/`pep257` lint tests were already failing before this pass (1171 lines, part of the wider lint debt tracked in `AUDIT.md`, rama `doc`) — out of scope for this driver-consolidation pass, not touched beyond removing the ~70 lines of the now-shared `PIDController`/`Agent` classes.
