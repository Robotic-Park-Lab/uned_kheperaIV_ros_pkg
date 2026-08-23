# uned_kheperaIV_ros_pkg

> 📖 Para entender las ramas de este repositorio y su guía de contribución, consulta la rama [`doc`](https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg/tree/doc).

Paquetes ROS 2 y ficheros de configuración para teleoperar y simular el robot móvil diferencial Khepera IV en ROS 2, Webots, Gazebo y Matlab. El objetivo es una herramienta Hardware-in-the-Loop fácil de escalar y mantener.

#### Estructura

- **doc**. Un fichero `.tex` con más detalle sobre el repositorio: diagramas ROS, bibliografía, enlaces de interés, etc.
- **scripts**. Ficheros auxiliares que no forman parte de ningún paquete ROS: los programas en C (`server.c`, `prog-template.c`) que corren en el propio Linux embebido del Khepera IV vía `libkhepera`, prototipos micro-ROS de Arduino/ESP32, y modelos de Matlab. Ver [scripts/README_es.md](scripts/README_es.md).
- **[uned_kheperaiv_config](uned_kheperaiv_config/README_es.md)**. Paquete ROS 2. Configuración del entorno: modelos 3D (incluyendo ahora lo que antes era el paquete independiente `uned_khepera_description`), el archivo de lanzamiento unificado `experience.launch.py`, y recursos de RViz/RQT.
- **[uned_kheperaiv_controllers](uned_kheperaiv_controllers/README_es.md)**. Paquete ROS 2. Controladores en C++ para docencia: control de posición PID periódico y PID basado en eventos, disparo de eventos. Portado desde la rama `benchmark`, donde se había desarrollado pero nunca se había incorporado al desarrollo activo.
- **[uned_kheperaiv_driver](uned_kheperaiv_driver/README_es.md)**. Paquete ROS 2. Nodo cliente TCP que habla con el programa en C que corre en el propio Linux embebido del Khepera IV.
- **[uned_kheperaiv_gazebo](uned_kheperaiv_gazebo/README_es.md)**. Paquete ROS 2 (antes `uned_khepera_gazebo`). Lanzamiento/plugins de simulación en Gazebo Classic 11 para el Khepera IV.
- **[uned_kheperaiv_gui](uned_kheperaiv_gui/README_es.md)**. Paquete ROS 2. Interfaz gráfica PyQt para manejar un único robot Khepera IV.
- **[uned_kheperaiv_task](uned_kheperaiv_task/README_es.md)**. Paquete ROS 2. Nodos de misión/tarea de alto nivel: control de formación basado en distancia y basado en forma, más un driver de formación específico para Gazebo.
- **[uned_kheperaiv_webots](uned_kheperaiv_webots/README_es.md)**. Paquete ROS 2. Driver virtual del Khepera IV en Webots.
- **[uned_vicon_gazebo](uned_vicon_gazebo/README_es.md)**. Paquete ROS 2. Puente de pose Vicon-a-Gazebo (ruta legacy, revisar una vez confirmada la actualización de Gazebo de arriba).

## Instalación :book:

El objetivo es [ROS 2 Humble Hawksbill](https://docs.ros.org/en/humble/index.html) en **Ubuntu 22.04**.

### Requisitos previos 📋

##### ROS 2
Instala primero ROS 2 Humble, siguiendo la [documentación oficial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

##### Webots (para `uned_kheperaiv_webots`)
```
sudo apt install ros-humble-webots-ros2-driver
```

##### Gazebo Classic (para `uned_kheperaiv_gazebo`, `uned_vicon_gazebo`)
```
sudo apt install ros-humble-gazebo-ros-pkgs
```
Gazebo Classic 11 es el simulador que empareja con ROS 2 Humble (la línea más nueva Gazebo/Ignition empareja con ROS 2 Jazzy en adelante, no usada en este repositorio).

##### `libkhepera` (para compilar el programa embarcado en `scripts/`)
`libkhepera` es el SDK propietario de K-Team SA para la plataforma Khepera. **No se incluye en este repositorio** — ningún fichero de licencia concede redistribución (ver `AUDIT.md` en la rama `doc`). Consíguelo por separado (de K-Team, o tu propia copia existente) y ver [scripts/README_es.md](scripts/README_es.md) para cómo apuntar la compilación hacia él.

##### Matlab
PENDIENTE — todavía no se ha fijado una versión concreta de Matlab/Simulink ni una lista de toolboxes. Ver [scripts/README_es.md](scripts/README_es.md).

##### Dependencias de otros repositorios del laboratorio
No se declaran con una clave de rosdep (no hay entrada en un índice público de rosdep) — clónalos y compílalos en el mismo workspace:
- **`multi_agent_pkg`**: de [Robotic-Park-Lab/RoboticPark](https://github.com/Robotic-Park-Lab/RoboticPark), requerido por `uned_kheperaiv_webots` para las matemáticas de formación multi-agente (multiplicadores de Lagrange para geometrías de esfera/cono/elipsoide).

### Compilar el workspace

```
mkdir -p ~/khepera_ws/src && cd ~/khepera_ws/src
git clone -b humble-dev https://github.com/Robotic-Park-Lab/uned_kheperaIV_ros_pkg
git clone -b humble-dev https://github.com/Robotic-Park-Lab/RoboticPark.git   # para multi_agent_pkg
cd ~/khepera_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Uso 🔧

### Lanzar una experiencia

Igual que el repositorio de Crazyflie, hay un **único archivo de lanzamiento parametrizado**, `uned_kheperaiv_config/launch/experience.launch.py`. Cada experiencia es un fichero `.yaml` en `uned_kheperaiv_config/resources/`:

```
ros2 launch uned_kheperaiv_config experience.launch.py config_file:=Demo_teleop_webots.yaml
```

| `config_file` | Descripción |
|---|---|
| `Demo_teleop_webots.yaml` | 1 Khepera IV virtual en Webots, teleoperado. |
| `Demo_formation_webots.yaml` | Control de formación distribuido en Webots. |
| `Demo_formation_central_webots.yaml` | Control de formación centralizado — referencia un ejecutable `centralized_formation_controller` que todavía no existe en `uned_kheperaiv_task`; el lanzamiento en sí ya no falla con este fichero (un bug real, corregido — ver `AUDIT.md`), pero la demo no funcionará por completo hasta que se escriba ese nodo. |

### Simuladores

- **Webots**: la ruta activamente mantenida — ver [uned_kheperaiv_webots/README_es.md](uned_kheperaiv_webots/README_es.md).
- **Gazebo Classic 11**: ver [uned_kheperaiv_gazebo/README_es.md](uned_kheperaiv_gazebo/README_es.md) para su estado actual — se está poniendo al día con la versión actual de Gazebo Classic 11 (la que empareja con ROS 2 Humble).

### Controlador Matlab
PENDIENTE

### Hardware-in-the-Loop
PENDIENTE: Micro-ROS

## Autores ✒️
* **[Francisco José Mañas Álvarez](https://github.com/FranciscoJManasAlvarez)** :envelope: fjmanas@dia.uned.es

## Publicaciones relacionadas :paperclip:
