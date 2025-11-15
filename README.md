# uned_kheperaIV_ros_pkg

EN DESARROLLO ...

Repositorio con los paquetes de ROS2 y ficheros de configuración para la teleoperación y simulación del robot móvil diferencial Khepera IV en ROS, Gazebo y Matlab. La finalidad es obtener una herramienta Hardware-in-the-Loop que sea facilmente escalable y mantenible.

#### Estructura 
- **doc**. Contiene un fichero _.tex_ que aborda más en detalle toda la información relacionada con el repositorio: esquemas de ROS, búsquedas bibliográficas, enlaces de interés, etc.
- **scripts**. Contiene aquellos ficheros auxiliares que no forman parte de ningún paquete de ROS. Por ejemplo, ficheros _.sh_ para automatizar procesos repetitivos como la conversión de los ficheros _.bag_ a txt o los scripts de Matlab para representar datasets.
- **uned_kheperaIV_config**. Paquete de ROS. Contiene aquellos elementos auxiliares para la configuración del entorno, así como los _.launch.py_ para la ejecución en bloque de las diferentes estructuras del sistema.

## Instalación :book:
El objetivo es implementar todo el sistema en [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) y [Ubuntu 22.04 LTS (Jammy Jellyfish)](https://releases.ubuntu.com/jammy/)  a fin de prolongar el mantenimiento y vigencia de la plataforma.

### Pre-requisitos 📋
##### ROS
Lo primero debe ser tener instalada la correspondiente versión de ROS para el sistema operativo del dispositivo ([Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)). 

##### Matlab


##### Dependencias


### - Ubuntu 22.04 - ROS Humble Hawksbill
La configuración del entorno de trabajo para el paquete desarrollado se muestra a continuación.
```
mkdir -p khepera_ws/src && cd khepera_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg 
cd ..
colcon build --symlink-install --packages-select uned_kheperaiv_config uned_kheperaiv_driver uned_khepera_description uned_kheperaiv_task uned_kheperaiv_webots roboticpark_config && source install/setup.bash
```

## Uso 🔧
```
ros2 launch uned_kheperaiv_config experience.launch.py config_file:=Demo_teleop_webots.yaml
```
### Simulador
TO-DO
Las simulaciones se hacen sobre Gazebo. El modelo sdf está operativo en el paquete uned_kheperaIV_config pero los plugins para los actuadores y sensores aún no se han actualizado. 
#### Exclusivo en ROS
TO-DO

#### Controlador en Matlab
TO-DO

### Hardware-in-the-Loop
TO-DO: Micro-ROS

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones asociadas :paperclip:
