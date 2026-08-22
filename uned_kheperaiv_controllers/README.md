# uned_kheperaiv_controllers

`ament_cmake` (C++) package with position controllers for the Khepera IV, meant as a teaching base. **Ported from the `benchmark` branch** (2026-08-22): it was developed there for a published book chapter but never carried over to active development — `humble-dev` had no equivalent to `uned_crazyflie_controllers` at all until now. `benchmark` itself was only read from, never modified.

## Controllers

| Executable | Class / header | Architecture |
|---|---|---|
| `periodic_pid_position_controller` | `PositionController` / `KheperaPositionController.hpp` | Periodic PID position control |
| `eventbased_pid_position_controller` | `PositionController` / `KheperaPositionController.hpp` | Event-based PID position control (relative threshold) |
| `event_triggering` | `EventTriggering` / `KheperaEventTriggering.hpp` | Event-triggering node |

Same pattern as `uned_crazyflie_controllers`: `periodic_pid_position_controller`/`eventbased_pid_position_controller` share the same `PositionController` header but each `.cpp` provides its own `initialize()`/control-loop implementation, compiled into separate executables.

## Adding a new technique

Follow the same pattern: a new header in `include/uned_kheperaiv_controllers/`, a new source in `src/`, register the executable in `CMakeLists.txt` (`add_executable` + `ament_target_dependencies` + `install(TARGETS ...)`).

## Dependencies

`rclcpp`, `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`.

## Tests

None yet — this package was only just ported from `benchmark`. Candidate for the same PID-math unit-testing treatment `uned_crazyflie_controllers` got (extracting the pure `pid_controller()` computation and testing it with `ament_add_gtest`, without needing a running node) — not done in this pass, flagged for follow-up.
