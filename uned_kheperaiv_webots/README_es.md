# uned_kheperaiv_webots

Paquete `ament_python` con el driver **virtual** del Khepera IV en [Webots](https://cyberbotics.com/).

## Estructura

- **`khepera_driver.py`** (`KheperaWebotsDriver`, cargado por `webots_ros2_driver` desde la etiqueta `<plugin>` del URDF del robot, no vía `ros2 run`): el driver simulado — movimiento, sensores, y control de formación (geometrías de distancia/punto/esfera/cono/elipsoide, vía `multi_agent_pkg.lagrange_multipliers`).
- Comparte `PIDController` y `Agent` con `uned_kheperaiv_driver` (el paquete driver del robot físico) — ver el README de ese paquete para saber qué se unificó y qué deliberadamente no. Este paquete solía importar `Agent` (y un `Crazyflie_ROS2` sin usar, código muerto) desde `uned_crazyflie_driver`, una dependencia real del repositorio de *Crazyflie* que este paquete nunca necesitó de verdad para nada específico de Crazyflie — eliminada, sustituida por la `Agent` propia de `uned_kheperaiv_driver`.

## Tests

No se han añadido tests funcionales en esta pasada: la lógica propia de `KheperaWebotsDriver` (`distance_gradient_controller`, `pose_gradient_controller`, `step()`, los bucles de control de seguimiento de geometría) necesita un objeto `Robot` de Webots real para ejecutarse de forma significativa, no disponible en este entorno. Las dos clases que ahora importa de `uned_kheperaiv_driver` (`PIDController`, `Agent`) **sí** están testeadas — ver la sección de Tests de `uned_kheperaiv_driver/README_es.md`.

**Preexistente, no corregido aquí**: los tests de lint `copyright`/`flake8`/`pep257` propios de `khepera_driver.py` ya fallaban antes de esta pasada (1171 líneas, parte de la deuda de lint más amplia rastreada en `AUDIT.md`, rama `doc`) — fuera del alcance de esta pasada de consolidación del driver, no tocado más allá de eliminar las ~70 líneas de las clases `PIDController`/`Agent` ahora compartidas.
