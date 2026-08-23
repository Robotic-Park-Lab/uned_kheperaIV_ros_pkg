# uned_kheperaiv_gazebo

Paquete `ament_cmake` con los archivos de lanzamiento y scripts de apoyo de la simulación en Gazebo Classic 11 para el Khepera IV (la ruta de simulador legacy -- `uned_kheperaiv_webots` es la que se mantiene activamente). Renombrado desde `uned_khepera_gazebo` por consistencia de nombres con el resto del repositorio.

> **Esta actualización a Gazebo Classic 11 la implementó por completo Claude (IA), como parte de una pasada automatizada de auditoría/reestructuración del repositorio, y todavía no ha sido validada por Francisco en su propio entorno o en una sesión real con la interfaz gráfica de Gazebo.** Todo lo de abajo se verificó con ejecuciones reales y headless de `gazebo`/`gzserver` y tráfico real de topics en el sandbox de esta sesión (ver "Qué se verificó realmente" más abajo) -- pero eso no es lo mismo que Francisco confirmando que funciona correctamente en su propio equipo. Trata este paquete como **pendiente de su revisión**, rastreado en `AUDIT.md` en la rama `doc`.

## Qué estaba roto (encontrado probando a lanzarlo de verdad, no solo leyendo el código)

- `multiple_robot_Gazebo.launch.py` / `demo_formation_Gazebo.launch.py` apuntaban a ficheros yaml/rviz que no existen (`demo_teleop.yaml`, `demo_formation.yaml`, `test.rviz`), y esperaban una clave `config_path` por robot que el esquema actual de `.yaml` de experiencia del repositorio (`Operation`/`Robots`/...) no tiene. `robot['pose'].split(', ')` tampoco coincidía con el formato de pose real de esos ficheros (separado por espacios, no por comas).
- `simple_launch.py` usa un **enfoque distinto, aparentemente nunca terminado**: `spawn_entity.py -database khepera_IV`, que busca un modelo SDF nativo de Gazebo por nombre vía `GAZEBO_MODEL_PATH`. Desajuste real confirmado: `uned_kheperaiv_config/model/model.config` nombra el modelo `Khepera-IV` (con mayúscula y guion), no `khepera_IV`, y `GAZEBO_MODEL_PATH` no está definida en ningún sitio de este repositorio. **Dejado tal cual, no corregido** -- es redundante con la ruta basada en URDF de abajo, que ya está verificada y funcionando; no merece la pena mantener dos enfoques paralelos de carga de modelo. Si quieres que esta demo concreta funcione, necesita `GAZEBO_MODEL_PATH` incluyendo `uned_kheperaiv_config/model/` y el nombre del modelo reconciliado.

## Qué se arregló y se verificó de verdad

`multiple_robot_Gazebo.launch.py` y `demo_formation_Gazebo.launch.py` ahora apuntan a los ficheros de experiencia reales y existentes (`Demo_teleop_webots.yaml`, `Demo_formation_webots.yaml` en `uned_kheperaiv_config/resources/`), leen correctamente su diccionario `Robots`, y separan `pose` por espacios en blanco. Un nuevo `uned_kheperaiv_config/resources/khepera_gazebo_default.yaml` aporta los valores por defecto `{task, communication}` por robot que `uned_kheperaiv_task/gazebo_driver.py` realmente lee (hace `documents[robot_id]`) -- antes no existía tal fichero, así que `gazebo_driver` no tenía nada válido que cargar. `task.enable` es `false` para todos los robots en ese nuevo fichero: cablea correctamente la infraestructura pero **no** activa el comportamiento de formación, ya que las relaciones/distancias reales entre robots no estaban especificadas en ningún sitio del repositorio e inventarlas no era seguro hacerlo a ciegas. Activar la formación para una demo real es trabajo futuro, no hecho aquí.

La **ruta URDF + plugin `gazebo_ros`** (`uned_kheperaiv_config/urdf/khepera.xml` -> `kheperaXX.urdf` generado, `libgazebo_ros_diff_drive.so` + `libgazebo_ros_p3d.so`) resultó ya usar sintaxis de plugin actual y funcional -- solo le faltaba tener algo correctamente cableado para lanzarla. Esta es la ruta que se conserva y se arregla; la ruta de modelo SDF de `simple_launch.py` es la que se deja como conocida-rota/legacy (ver arriba).

### Qué se verificó realmente (comandos ejecutados de verdad en esta sesión)

```
gazebo --minimal_comms -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
  <share>/uned_kheperaiv_config/worlds/UNED_RoboticParkLab_invert.world
ros2 run uned_kheperaiv_gazebo inject_entity.py <share>/uned_kheperaiv_config/urdf/khepera01.urdf 0.0 0.0 0.05 0
# -> SpawnEntity success=True, aparecen los topics /khepera01/{cmd_vel,ground_truth,odom}
ros2 topic pub /khepera01/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 20
# -> /khepera01/ground_truth position.x se movió de 0.0016 a 0.661 en ~3s -- movimiento real, no solo existencia del topic
```
Repetido con los 4 robots de `Demo_formation_webots.yaml` (`khepera01`-`khepera04`) apareciendo simultáneamente con topics correctamente namespaced para cada uno.

## Estructura

- `launch/multiple_robot_Gazebo.launch.py`, `launch/demo_formation_Gazebo.launch.py`: verificados funcionando (spawn + movimiento básico), la tarea de formación en sí no activada (ver arriba).
- `launch/simple_launch.py`: conocido como roto (búsqueda de modelo en base de datos SDF), no corregido, ver arriba.
- `src/inject_entity.py`: hace spawn de un fichero URDF/SDF en Gazebo vía el servicio `SpawnEntity`. Sin modificar.
- `worlds/Empty.world`: no usado por los dos archivos de lanzamiento funcionales (usan en su lugar `uned_kheperaiv_config/worlds/UNED_RoboticParkLab_invert.world`) -- conservado, no investigado más.
- `rviz/test.rviz`: **no existe** pese a estar referenciado por los archivos de lanzamiento previos a la corrección -- el arreglo apuntó ambos archivos de lanzamiento a `uned_kheperaiv_config/rviz/default.rviz` en su lugar, que sí existe y es YAML de RViz válido, pero su configuración real de displays no se ha re-verificado contra los topics de este paquete.

## Tests

No se han añadido tests funcionales -- este paquete es código de lanzamiento/pegamento para Gazebo, su corrección es lo que comprueban los comandos "verificado realmente" de arriba, no algo testeable unitariamente de forma significativa sin una instancia de Gazebo en ejecución.
