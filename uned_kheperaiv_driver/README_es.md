# uned_kheperaiv_driver

Paquete `ament_python` con el driver para robots Khepera IV **físicos**, y el código base compartido con `uned_kheperaiv_webots` (el driver **virtual**/Webots).

## Estructura

- **`kheperaIV_client_driver.py`** (punto de entrada `kheperaIV_client_driver`): `KheperaIVDriver`, un cliente TCP que habla con el programa en C que corre en el propio Linux embebido del robot (`scripts/server.c` en la raíz del repositorio, compilado contra `libkhepera` — ver el README raíz y `scripts/Makefile`). Un nodo por robot físico, lanzado con `namespace=<robot_id>`.
- **`pid_controller.py`**: `PIDController`, compartido con `uned_kheperaiv_webots/khepera_driver.py` — extraído porque era idéntico byte a byte en ambos ficheros (verificado con un `diff` real antes de extraerlo, no asumido).
- **`agent.py`**: `Agent`, usado por `uned_kheperaiv_webots/khepera_driver.py` para el seguimiento de vecinos en control de formación (objetivos de distancia/punto/geometría, marcadores de RViz). Antes se importaba desde `uned_crazyflie_driver` — una dependencia real entre repositorios que obligaba a tener clonado y compilado el repositorio de Crazyflie en el mismo workspace solo para el control de formación del Khepera. Sustituido por esta copia propia del Khepera, adaptada de la versión de Crazyflie (sus ramas `high_level_commander`/`scf` — específicas del vuelo del Crazyflie, nunca alcanzables desde el driver Webots del Khepera — se eliminaron, no se arrastraron a ciegas).

### Dos clases `Agent`, a propósito

`kheperaIV_client_driver.py` tiene su **propia** clase `Agent` (distinta de la de este módulo), que habla con el robot físico a través del protocolo TCP real (`self.sock.sendall(...)`, comandos `"n <id> <d> <k>"`/`"m <id> <x> <y> <z>"` que entiende `scripts/server.c`). **No** está unificada con la `Agent` de `agent.py`: ambas resuelven problemas genuinamente distintos (protocolo de socket real frente a seguimiento simulado solo-ROS), y fusionarlas cambiaría comportamiento real de cara al hardware que no se puede verificar sin un robot físico. `PIDController` sí era seguro unificarlo porque era un duplicado demostrable, idéntico byte a byte; `Agent` no lo es.

## Tests

- `test_pid_controller.py`: comportamiento real de `PIDController.update()`/`eval_threshold()` — respuesta proporcional, saturación en los límites configurados, la peculiaridad real de que `UpperLimit == 0.0` desactiva la saturación por completo (no "saturar a cero"), que el término integral usa el error de la llamada *anterior*, y el disparo basado en eventos.
- `test_agent.py`: `Agent` contra un nodo/padre ROS falso mínimo (con `create_subscription`/`create_publisher`/`get_logger`/`get_clock` de pato) — el modo distancia se suscribe al topic correcto, el modo línea calcula bien el módulo del vector, un padre `digital_twin` omite los publicadores de marcador, `gtpose_callback` actualiza la pose seguida sin fallar.
- **No testeado**: `KheperaIVDriver` (socket TCP real a hardware) y la propia clase `Agent` de `kheperaIV_client_driver.py` — ambas necesitan un Khepera real o al menos un servidor de socket para ejercerse de forma significativa; no intentado aquí para evitar un test frágil que simule todo el protocolo en lugar de testear algo real.
- **Preexistente, no corregido aquí**: los tests de lint `flake8`/`pep257` propios de `kheperaIV_client_driver.py` ya fallaban antes de esta pasada (parte de la deuda de lint más amplia rastreada en `AUDIT.md`, rama `doc`) — fuera del alcance de esta pasada de consolidación del driver, no tocado.
