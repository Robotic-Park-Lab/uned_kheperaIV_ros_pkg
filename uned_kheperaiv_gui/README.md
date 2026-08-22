# uned_kheperaiv_gui

`ament_python` package with the PyQt5 graphical interface for handling a single Khepera IV robot.

## Structure

- `interface_gui.py` (entry point `interface_node`): the real, working `MainWindow`. Deliberately basic: it loads `main.ui` as-is and does not embed `rqt_robot_steering`/`rqt_plot`/`rqt_graph` panels — see "Future work" below for why and what the original code attempted. Previously there was also an `interface_node.py` stub registered as the actual entry point (it only printed `"Hi from uned_kheperaiv_gui."`) while this file sat unused and broken (`from main_ui import *`, an absolute import outside the package); the stub has been removed and this file fixed and wired up for real.
- `main.ui` / `main_ui.py`: the window layout (Qt Designer) and its compiled-to-Python version. Only `main.ui` is actually used at runtime (loaded directly via `uic.loadUi`); `main_ui.py` (the compiled `Ui_MainWindow` class) is not imported anywhere — it looks like a stale generated artifact kept for reference, not deleted here since removing generated files that might still be wanted wasn't asked for.
- `logo.qrc` / `logo_rc.py`: Qt resources (logos), compiled from `logo.qrc`.
- `shell_cmd.py`: a small subprocess-tracking helper (`ShellCmd`). Not used by `interface_gui.py` in its current basic form; kept as-is (it's self-contained and harmless), lint issues in it fixed as a side effect of getting this package's lint tests green.

## Regenerating the compiled Qt files

If you edit `main.ui` or `logo.qrc`:
```
pyuic5 -x main.ui -o main_ui.py
pyrcc5 -o logo_rc.py logo.qrc
```

## Usage

```
cd dev_ws
colcon build --symlink-install --packages-select uned_kheperaiv_gui
ros2 run uned_kheperaiv_gui interface_node
```

`main.ui` and the `figs/` images are declared as `package_data` in `setup.py` so they install correctly in every build mode, not just `--symlink-install` (verified with a real non-symlink `colcon build`: both end up next to `interface_gui.py` in the installed package, exactly where the code expects them).

## Future work

The original `interface_gui.py` embedded `rqt_robot_steering` (as an "Open Loop" tab), plus `rqt_plot`/`rqt_graph`/a camera view, directly inside the main window — by spawning each as a subprocess, finding its X11 window ID with `xdotool`, and re-parenting it via `QWindow.fromWinId()`. That only works under X11 (not Wayland), and `xdotool` was never declared as a dependency anywhere in this package. The exact same tradeoff was already made for `uned_crazyflie_gui` in the sibling Crazyflie repo: keep a basic, honestly-working window now, and leave the fuller embedded-panels vision as a documented possibility rather than a currently-broken feature. Re-implementing it (or replacing it with a cleaner mechanism, e.g. running `rqt` as separate windows instead of embedding them) is open work.

## Tests

- `test_copyright.py` / `test_flake8.py` / `test_pep257.py`: standard `ament` lint, with `main_ui.py`/`logo_rc.py` excluded (generated code, not hand-written — same pattern used for the equivalent files in `uned_crazyflie_gui`). All pass on real hand-written code.
- `test_interface_gui.py` (new): a real headless smoke test (`QT_QPA_PLATFORM=offscreen`) that constructs the actual `MainWindow` with a real `rclpy` node and checks it starts without raising — the same class of regression the pre-fix `from main_ui import *` bug was, so a broken import or a `main.ui` that fails to resolve at runtime would be caught immediately instead of only failing interactively. It does not test UI interaction/widget behavior, only that the window builds successfully.
