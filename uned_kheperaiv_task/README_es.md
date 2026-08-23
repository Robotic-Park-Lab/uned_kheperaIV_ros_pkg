# uned_kheperaiv_task

Paquete `ament_python` con nodos de misión/tarea de alto nivel para el Khepera IV: tres drivers de control de formación, uno por ruta de simulación/comunicación.

## Nodos

- **`distance_based_formation_control`**: control de formación basado en distancia sobre el contrato de topics estilo Webots (`<id>/local_pose` de entrada, `goal_pose` de salida). Cada vecino se sigue con un `Agent` que publica un `Marker` de RViz con código de color mostrando cuánto se acerca la distancia inter-robot medida a la distancia objetivo.
- **`shape_based_formation_control`**: control de formación basado en forma (offsets x/y fijos por vecino en vez de una distancia objetivo) sobre el mismo tipo de contrato de topics. Más simple que los otros dos -- su `Agent` solo sigue la pose de un vecino, sin publicar marcador/error.
- **`gazebo_driver`**: el driver de formación específico para Gazebo. A diferencia de los dos anteriores, gestiona por sí mismo toda la pila de control: se suscribe a `nav_msgs/Odometry` (no una `Pose` simple), difunde TF, ejecuta un bucle PID interno de IPC (velocidad lineal/angular), y publica `cmd_vel` directamente -- no depende de un controlador de posición separado aguas abajo. También soporta un tipo de robot `digital_twin`. Este nodo se conserva y se está actualizando para que Gazebo vuelva a funcionar (ver `uned_kheperaiv_gazebo` y los recursos de Gazebo en `uned_kheperaiv_config`, trabajados por separado) -- **no** es legacy/obsoleto.

## Qué se deduplicó, y qué no

Los tres nodos originalmente definían su propia clase `Agent`, y `gazebo_driver.py` también definía su propio `PIDController`. Tras inspeccionarlo (no solo por el nombre -- se leyeron los tres ficheros completos):

- **`PIDController`** (usado hoy solo por `gazebo_driver.py`): extraído literalmente a `pid_controller.py`. Lógica real, que merece de verdad su propio módulo testeable incluso sin duplicación entre nodos dentro de este paquete. **Nota para quien trabaje después en `uned_kheperaiv_webots`**: su `khepera_driver.py` tiene (o está a punto de tener, otra sesión trabajaba en paralelo en ello) su propio `PIDController` también -- merece la pena comprobar si tiene la misma forma y unificarlo más, no hecho aquí para mantener el alcance acotado.
- **La mitad de dibujo de marcador de `Agent.gtpose_callback`** en `distance_based_formation_control.py` y `gazebo_driver.py`: matemáticas y umbrales genuinamente idénticos (0.05 m rojo / 0.025 m naranja / si no, verde), solo con nombres de atributo distintos en el nodo padre (`self.groundtruth`/`self.distance` frente a `self.gt_pose`/`self.d`) y la comprobación extra `digital_twin` de gazebo. Extraído a una función pura, `formation_marker.build_distance_marker()`, llamada desde ambos. Verificado que la extracción reproduce el comportamiento original con `test/test_formation_marker.py` (umbrales de color y geometría de línea), no solo comprobado a ojo.
- **Las tres clases `Agent`/`KheperaIVDriver` en sí *no* se unificaron.** La auditoría que señaló esta duplicación tenía razón en que las tres definen clases con los mismos nombres, pero leer los cuerpos completos muestra que no son en realidad el mismo algoritmo con diferencias cosméticas -- son tres estrategias de formación distintas, con firmas de constructor distintas, distintos tipos de mensaje suscritos, y (para `gazebo_driver`) una arquitectura de control completamente distinta (bucle PID interno propio + `cmd_vel`, frente a los otros dos, que solo publican un `goal_pose` para que otra cosa lo siga). Forzar su unificación habría significado inventar una abstracción compartida que ninguno de los tres tiene realmente, un resultado peor que la duplicación actual y honesta. Se dejaron como tres ficheros de nodo separados.

## Tests

- `test/test_formation_marker.py`: 4 tests sobre `build_distance_marker` (umbrales rojo/naranja/verde, geometría de línea) -- función pura, no necesita `rclpy.init()`.
- `test/test_pid_controller.py`: 6 tests sobre `PIDController` (salida solo-proporcional, saturación en ambos límites, saturación desactivada cuando `UpperLimit == 0.0`, el integral acumula el error *anterior* y no el actual, lógica de umbral del disparo basado en eventos).
- No testeadas, y no razonablemente testeables sin mucho más trabajo: las tres clases de nodo `KheperaIVDriver` en sí (nodos `rclpy` reales con cableado de topics/parámetros/temporizadores) y las mitades de cara a ROS de las clases `Agent` (crean suscripciones/publicadores reales en `__init__`).

## Al extraer: limpieza de lint y hallazgos señalados-no-corregidos

Al tocar estos ficheros para llegar a un `colcon test` limpio, también se corrigieron fallos de lint preexistentes no relacionados con la deduplicación en sí (cabeceras de copyright ausentes -- faltaban en todo el repositorio antes de esta pasada, solo añadidas aquí para los ficheros de este paquete; imports sin usar `PoseWithCovariance`, un import duplicado de `math.sqrt`, `tf_transformations`, `radians`/`pi` en `gazebo_driver.py`; varios problemas de estilo `E501`/`E231`/`E225`). Dos carencias reales se encontraron y **se señalaron con `# noqa` + un comentario en vez de corregirse**, ya que corregirlas significa cambiar matemáticas de control reales sin poder validarlas contra hardware/simulación:

- `L` (separación entre ruedas) de `IPC_controller` se declara pero nunca se usa en la cinemática -- parece que debía alimentar la fórmula y no lo hace.
- `delta` de `dt_pose_callback` (offset entre la posición del gemelo digital y la real) se calcula pero nunca se aplica en ningún sitio -- parece una corrección de gemelo digital incompleta.
