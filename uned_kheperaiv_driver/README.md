# uned_kheperaiv_driver

`ament_python` package with the driver for **physical** Khepera IV robots, and the base code shared with `uned_kheperaiv_webots` (the **virtual**/Webots driver).

## Structure

- **`kheperaIV_client_driver.py`** (entry point `kheperaIV_client_driver`): `KheperaIVDriver`, a TCP client that talks to the C program running on the robot's own onboard embedded Linux (`scripts/server.c` at the repo root, built against `libkhepera` — see the root README and `scripts/Makefile`). One node per physical robot, launched with `namespace=<robot_id>`.
- **`pid_controller.py`**: `PIDController`, shared with `uned_kheperaiv_webots/khepera_driver.py` — extracted because it was byte-for-byte identical in both files (verified with a real `diff` before extracting, not assumed).
- **`agent.py`**: `Agent`, used by `uned_kheperaiv_webots/khepera_driver.py` for neighbour tracking in formation control (distance/point/geometry targets, RViz markers). This used to be imported from `uned_crazyflie_driver` — a real cross-repo dependency that made this repo require the Crazyflie repo cloned and built in the same workspace just for Khepera formation control. Replaced with this Khepera-owned copy, adapted from the Crazyflie version (its `high_level_commander`/`scf` branches — Crazyflie-flight-specific, never reachable from the Webots Khepera driver — were dropped, not carried over blindly).

### Two `Agent` classes, on purpose

`kheperaIV_client_driver.py` has its **own**, separate `Agent` class (not this module's), which talks to the physical robot over the real TCP socket protocol (`self.sock.sendall(...)`, `"n <id> <d> <k>"`/`"m <id> <x> <y> <z>"` commands understood by `scripts/server.c`). It is **not** unified with `agent.py`'s `Agent` here: the two solve genuinely different problems (real socket protocol vs. ROS-only simulated tracking), and merging them would change real hardware-facing behavior that can't be verified without a physical robot. `PIDController` *was* safe to unify because it was a provable, byte-identical duplicate; `Agent` is not.

## Tests

- `test_pid_controller.py`: real behavior of `PIDController.update()`/`eval_threshold()` — proportional response, saturation at the configured limits, the real quirk that `UpperLimit == 0.0` disables saturation entirely (not "clamp to zero"), that the integral term uses the *previous* call's error, and event-based triggering.
- `test_agent.py`: `Agent` against a minimal fake ROS node/parent (duck-typed `create_subscription`/`create_publisher`/`get_logger`/`get_clock`) — distance-mode subscribes to the right topic, line-mode computes the vector modulus correctly, a `digital_twin` parent skips the marker publishers, `gtpose_callback` updates the tracked pose without crashing.
- **Not tested**: `KheperaIVDriver` (real TCP socket to hardware) and `kheperaIV_client_driver.py`'s own `Agent` class — both need a real Khepera or at least a socket server to exercise meaningfully; not attempted here to avoid a fragile test that mocks the entire protocol instead of testing anything real.
- **Pre-existing, not fixed here**: `kheperaIV_client_driver.py`'s own `flake8`/`pep257` lint tests were already failing before this pass (part of the wider lint debt tracked in `AUDIT.md`, rama `doc`) — out of scope for this driver-consolidation pass, not touched.
