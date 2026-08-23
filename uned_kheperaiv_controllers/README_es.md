# uned_kheperaiv_controllers

Paquete `ament_cmake` (C++) con controladores de posición para el Khepera IV, pensado como base docente. **Portado desde la rama `benchmark`** (2026-08-22): se había desarrollado allí para un capítulo de libro publicado pero nunca se incorporó al desarrollo activo — `humble-dev` no tenía ningún equivalente a `uned_crazyflie_controllers` hasta ahora. `benchmark` en sí solo se ha leído, nunca modificado.

## Controladores

| Ejecutable | Clase / cabecera | Arquitectura |
|---|---|---|
| `periodic_pid_position_controller` | `PositionController` / `KheperaPositionController.hpp` | Control de posición PID periódico |
| `eventbased_pid_position_controller` | `PositionController` / `KheperaPositionController.hpp` | Control de posición PID basado en eventos (umbral relativo) |
| `event_triggering` | `EventTriggering` / `KheperaEventTriggering.hpp` | Nodo de disparo de eventos |

Mismo patrón que `uned_crazyflie_controllers`: `periodic_pid_position_controller`/`eventbased_pid_position_controller` comparten la misma cabecera `PositionController`, pero cada `.cpp` aporta su propia implementación de `initialize()`/bucle de control, compilada en ejecutables separados.

## Añadir una nueva técnica

Sigue el mismo patrón: una nueva cabecera en `include/uned_kheperaiv_controllers/`, una nueva fuente en `src/`, registrar el ejecutable en `CMakeLists.txt` (`add_executable` + `ament_target_dependencies` + `install(TARGETS ...)`).

## Dependencias

`rclcpp`, `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`.

## Tests

Ninguno todavía — este paquete se acaba de portar desde `benchmark`. Candidato al mismo tratamiento de test unitario de las matemáticas del PID que recibió `uned_crazyflie_controllers` (extraer el cálculo puro de `pid_controller()` y testearlo con `ament_add_gtest`, sin necesitar un nodo en ejecución) — no hecho en esta pasada, señalado como trabajo futuro.
