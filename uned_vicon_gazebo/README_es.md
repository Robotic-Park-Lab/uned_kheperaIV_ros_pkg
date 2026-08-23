# uned_vicon_gazebo

Paquete `ament_python` con un puente de pose Vicon-a-Gazebo para el Khepera IV (ruta legacy de Gazebo — revisar una vez confirmada por Francisco la actualización a Gazebo Classic 11 de `uned_kheperaiv_gazebo`, ver el README de ese paquete).

## Estructura

- **`vicon_gazebo.py`** (punto de entrada `vicon_gazebo`): `ViconGazebo`, un nodo que instancia una `Agent` por cada id de robot listado en su parámetro `agents` (separados por comas, p. ej. `khepera01, khepera02`). Cada `Agent` se suscribe a `<id>/ground_truth` (`nav_msgs/Odometry`, la propia salida ground-truth de Gazebo) y republica solo la pose en `<id>/pose` (`geometry_msgs/Pose`) — es decir, adapta el ground truth simulado de Gazebo al mismo tipo de topic `Pose` que publicaría un puente Vicon real, para que los nodos aguas abajo no necesiten saber si están funcionando contra hardware Vicon real o una simulación de Gazebo.
- El nodo también declara una suscripción `topic`/`String` sin usar (`listener_callback`) — resto de la plantilla de ejemplo de `ros2 pkg create`, nunca eliminada. No tocada en esta pasada (ver `AUDIT.md` en la rama `doc`).

## Uso

```
ros2 run uned_vicon_gazebo vicon_gazebo --ros-args -p agents:="khepera01, khepera02"
```

## Tests

Ninguno más allá de los tests de lint estándar `ament_copyright`/`ament_flake8`/`ament_pep257`, que tienen fallos preexistentes (ver `AUDIT.md` en la rama `doc`, sección 2 — la deuda de lint de `uned_vicon_gazebo` se señaló, no se corrigió, en esta pasada). `ViconGazebo`/`Agent` son nodos `rclpy` reales con suscripciones/publicadores cableados directamente desde `__init__`; testearlos de forma significativa necesita un grafo ROS en ejecución (o el propio Gazebo), no intentado aquí.
