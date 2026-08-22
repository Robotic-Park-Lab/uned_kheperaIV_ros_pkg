# uned_kheperaiv_task

`ament_python` package with high-level mission/task nodes for the Khepera IV: three formation-control drivers, one per simulation/communication path.

## Nodes

- **`distance_based_formation_control`**: distance-based formation control over the Webots-style topic contract (`<id>/local_pose` in, `goal_pose` out). Each neighbour is tracked by an `Agent` that publishes a colour-coded RViz `Marker` showing how close the measured inter-robot distance is to the target.
- **`shape_based_formation_control`**: shape-based formation control (fixed x/y offsets per neighbour instead of a target distance) over the same kind of topic contract. Simpler than the other two -- its `Agent` only tracks a neighbour's pose, no marker/error publishing.
- **`gazebo_driver`**: the Gazebo-specific formation driver. Unlike the two above, it owns the full control stack itself: subscribes to `nav_msgs/Odometry` (not a bare `Pose`), broadcasts TF, runs an inner IPC (linear/angular velocity) PID loop, and publishes `cmd_vel` directly -- it doesn't rely on a separate position controller downstream. Also supports a `digital_twin` robot type. This node is being kept and is getting Gazebo updated to work again (see `uned_kheperaiv_gazebo` and the Gazebo assets in `uned_kheperaiv_config`, worked on separately) -- it is **not** legacy/deprecated.

## What was deduplicated, and what wasn't

All three nodes originally defined their own `Agent` class, and `gazebo_driver.py` also defined its own `PIDController`. On inspection (not just by name -- read all three files in full):

- **`PIDController`** (used only by `gazebo_driver.py` today): extracted verbatim to `pid_controller.py`. Real logic, genuinely worth its own testable module even without cross-node duplication within this package. **Note for whoever works on `uned_kheperaiv_webots` next**: its `khepera_driver.py` has (or is getting, another session was working on it in parallel) its own `PIDController` too -- worth checking if it's the same shape and unifying further, not done here to stay in scope.
- **The marker-drawing half of `Agent.gtpose_callback`** in `distance_based_formation_control.py` and `gazebo_driver.py`: genuinely identical math and thresholds (0.05 m red / 0.025 m orange / else green), just different attribute names on the parent node (`self.groundtruth`/`self.distance` vs `self.gt_pose`/`self.d`) and gazebo's extra `digital_twin` gate. Extracted to a pure function, `formation_marker.build_distance_marker()`, called from both. Verified the extraction reproduces the original behaviour with `test/test_formation_marker.py` (color thresholds and line geometry), not just eyeballed.
- **The three `Agent`/`KheperaIVDriver` classes themselves were *not* unified.** The audit that flagged this duplication was right that all three define classes with the same names, but reading the full bodies shows they're not actually the same algorithm with cosmetic differences -- they're three different formation strategies with different constructor signatures, different subscribed message types, and (for `gazebo_driver`) an entirely different control architecture (own PID inner loop + `cmd_vel`, vs. the other two which just publish a `goal_pose` for something else to track). Force-unifying them would mean inventing a shared abstraction none of them actually share, which is a worse outcome than the current, honest duplication. Left as three separate node files.

## Tests

- `test/test_formation_marker.py`: 4 tests on `build_distance_marker` (red/orange/green thresholds, line geometry) -- pure function, no `rclpy.init()` needed.
- `test/test_pid_controller.py`: 6 tests on `PIDController` (proportional-only output, saturation at both limits, saturation disabled when `UpperLimit == 0.0`, integral accumulates the *previous* error not the current one, event-triggering threshold logic).
- Not tested, and not reasonably testable without much more work: the three `KheperaIVDriver` node classes themselves (real `rclpy` nodes with topic/parameter/timer wiring) and the `Agent` classes' ROS-facing halves (they create real subscriptions/publishers in `__init__`).

## While extracting: lint cleanup and flagged-not-fixed findings

While touching these files to reach a clean `colcon test`, also fixed pre-existing lint failures unrelated to the deduplication itself (missing copyright headers -- these were missing repo-wide before this pass, only added here for files in this package; unused imports `PoseWithCovariance`, a duplicate `math.sqrt` import, `tf_transformations`, `radians`/`pi` in `gazebo_driver.py`; various `E501`/`E231`/`E225` style issues). Two real gaps found and **flagged with `# noqa` + a comment instead of fixed**, since fixing them means changing actual control math without being able to validate it against hardware/simulation:

- `IPC_controller`'s `L` (wheel separation) is declared but never used in the kinematics -- looks like it was meant to feed the formula and doesn't.
- `dt_pose_callback`'s `delta` (digital-twin vs. real position offset) is computed but never applied anywhere -- looks like an incomplete digital-twin correction.
