# uned_kheperaiv_gui

Paquete `ament_python` con la interfaz gráfica PyQt5 para manejar un único robot Khepera IV.

## Estructura

- `interface_gui.py` (punto de entrada `interface_node`): la `MainWindow` real y funcional. Deliberadamente básica: carga `main.ui` tal cual y no embebe paneles `rqt_robot_steering`/`rqt_plot`/`rqt_graph` — ver "Trabajo futuro" más abajo para el porqué y qué intentaba el código original. Antes también existía un `interface_node.py`, un stub registrado como el punto de entrada real (solo imprimía `"Hi from uned_kheperaiv_gui."`) mientras este fichero permanecía sin usar y roto (`from main_ui import *`, un import absoluto fuera del paquete); el stub se ha eliminado y este fichero se ha corregido y conectado de verdad.
- `main.ui` / `main_ui.py`: el diseño de la ventana (Qt Designer) y su versión compilada a Python. Solo `main.ui` se usa realmente en tiempo de ejecución (cargado directamente vía `uic.loadUi`); `main_ui.py` (la clase compilada `Ui_MainWindow`) no se importa en ningún sitio — parece un artefacto generado obsoleto conservado como referencia, no eliminado aquí ya que borrar ficheros generados que quizá todavía se quieran no era parte de lo pedido.
- `logo.qrc` / `logo_rc.py`: recursos Qt (logotipos), compilados desde `logo.qrc`.
- `shell_cmd.py`: un pequeño ayudante de seguimiento de subprocesos (`ShellCmd`). No usado por `interface_gui.py` en su forma básica actual; conservado tal cual (es autocontenido e inofensivo), sus problemas de lint corregidos como efecto colateral de dejar en verde los tests de lint de este paquete.

## Regenerar los ficheros Qt compilados

Si editas `main.ui` o `logo.qrc`:
```
pyuic5 -x main.ui -o main_ui.py
pyrcc5 -o logo_rc.py logo.qrc
```

## Uso

```
cd dev_ws
colcon build --symlink-install --packages-select uned_kheperaiv_gui
ros2 run uned_kheperaiv_gui interface_node
```

`main.ui` y las imágenes de `figs/` se declaran como `package_data` en `setup.py` para que se instalen correctamente en cualquier modo de compilación, no solo `--symlink-install` (verificado con un `colcon build` real sin symlink: ambos acaban junto a `interface_gui.py` en el paquete instalado, exactamente donde el código los espera).

## Trabajo futuro

El `interface_gui.py` original embebía `rqt_robot_steering` (como pestaña "Open Loop"), más `rqt_plot`/`rqt_graph`/una vista de cámara, directamente dentro de la ventana principal — lanzando cada uno como subproceso, encontrando su ID de ventana X11 con `xdotool`, y reparentándolo vía `QWindow.fromWinId()`. Eso solo funciona bajo X11 (no Wayland), y `xdotool` nunca se declaró como dependencia en ningún sitio de este paquete. Exactamente la misma decisión se tomó ya para `uned_crazyflie_gui` en el repositorio hermano de Crazyflie: mantener ahora una ventana básica y honestamente funcional, y dejar la visión completa de paneles embebidos como una posibilidad documentada en vez de una funcionalidad actualmente rota. Reimplementarla (o sustituirla por un mecanismo más limpio, p. ej. lanzar `rqt` como ventanas separadas en vez de embeberlas) es trabajo abierto.

## Tests

- `test_copyright.py` / `test_flake8.py` / `test_pep257.py`: lint estándar de `ament`, con `main_ui.py`/`logo_rc.py` excluidos (código generado, no escrito a mano — mismo patrón usado para los ficheros equivalentes en `uned_crazyflie_gui`). Todos pasan sobre el código real escrito a mano.
- `test_interface_gui.py` (nuevo): un smoke test headless real (`QT_QPA_PLATFORM=offscreen`) que construye la `MainWindow` real con un nodo `rclpy` real y comprueba que arranca sin lanzar una excepción — la misma clase de regresión que era el bug `from main_ui import *` previo a la corrección, así que un import roto o un `main.ui` que falle al resolverse en tiempo de ejecución se detectaría de inmediato en vez de fallar solo de forma interactiva. No testea interacción/comportamiento de la interfaz, solo que la ventana se construye con éxito.
