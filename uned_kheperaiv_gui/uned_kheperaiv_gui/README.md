## Info

## Instalación :book:
```
pyuic5 -x main.ui -o main_ui.py
pyrcc5 -o logo_rc.py logo.qrc
```

### Pre-requisitos 📋
##### PyQt5

##### Dependencias


## Uso 🔧
### Variables
#### ROS2
```
cd \dev_ws
colcon build --symlink-install --packages-select uned_crazyflie_gui
ros2 run uned_crazyflie_gui interface_node
```

```
ros2 launch uned_crazyflie_config test.launch.py
```

