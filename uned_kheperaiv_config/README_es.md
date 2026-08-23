# uned_kheperaiv_config

Paquete `ament_cmake` con la configuración de entorno compartida por el resto de este repositorio: modelos 3D, el archivo de lanzamiento unificado, y recursos de RViz/RQT.

## Estructura

- **`urdf/`**, **`meshes/`**, **`src/replace.py`**: la descripción URDF del Khepera IV — fusionada desde el antiguo paquete independiente `uned_khepera_description` (no merecía la pena mantenerlo aparte, la misma decisión que la absorción de `uned_crazyflie_common` en `uned_crazyflie_driver` en el repositorio de Crazyflie). `urdf/khepera.xml` es la plantilla origen (placeholders `${suffix}`/`${topic_ns}`); la compilación genera `kheperaXX.urdf` para X de 01 a 40 vía `src/replace.py`, instalado junto a los 3 ficheros URDF origen. `khepera.xml` incluye los plugins de Gazebo Classic (`libgazebo_ros_diff_drive.so`, `libgazebo_ros_p3d.so`) usados por `uned_kheperaiv_gazebo`.
- **`model/`**: un modelo SDF alternativo, nativo de Gazebo, del Khepera IV (`model.sdf`/`model.config`, pensado para el flujo de spawn desde la base de datos de modelos de Gazebo vía `spawn_entity.py -database`) — ver `uned_kheperaiv_gazebo/README_es.md` para saber cuál de los dos enfoques de modelo (este vs. el URDF de arriba) es el que realmente se mantiene funcionando.
- **`launch/experience.launch.py`**: el único archivo de lanzamiento parametrizado — un `ros2 launch uned_kheperaiv_config experience.launch.py config_file:=<experiencia>.yaml` en lugar de un `.launch.py` por demo. Lee las secciones `Operation`/`Experience`/`Architecture`/`CPU_Monitoring`/`Interface`/`Data_Logging`/`Robots`/`Supervisor`/`Other` de un `.yaml` en `resources/`. Ver `resources/Demo_teleop_webots.yaml` para un ejemplo completo ya resuelto.
- **`launch/urdf_visualize.launch.py`**: visor URDF independiente en RViz (del antiguo `uned_khepera_description`).
- **`resources/`**: un `.yaml` por experiencia (ver la tabla en el README raíz), más `vicon_config.yaml`.
- **`rviz/`**, **`rqt/`**: configuraciones de RViz y perspectivas de RQT ligadas a demos concretas.
- **`worlds/`**: mundos de simulación de Webots (`.wbt`) y Gazebo (`.world`).

## Dependencias de otros repositorios del laboratorio

`uned_kheperaiv_webots` necesita `multi_agent_pkg` (de `RoboticPark`) compilado en el mismo workspace — ver la sección de Instalación del README raíz.
