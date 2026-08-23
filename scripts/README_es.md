# Scripts

Ficheros auxiliares que no forman parte de ningún paquete ROS 2.

## Programas embarcados del Khepera IV (`server.c`, `prog-template.c`)

Programas en C que corren en el propio Linux embebido del Khepera IV, compilados contra [`libkhepera`](https://www.k-team.com/) (el SDK de Khepera de K-Team SA — **no incluido en este repositorio**, ver la sección "Instalación" del README raíz y `AUDIT.md` en la rama `doc` para saber por qué). Abren un servidor TCP (puerto 50000) al que se conecta como cliente el nodo `kheperaIV_client_driver` de `uned_kheperaiv_driver`.

`prog-template.c` ("Código optimizado para Khepera IV - Comunicación ROS2", de Francisco José Mañas Álvarez, 05-2025) es la versión más nueva y completa — control de motores con interpolación suave, TCP no bloqueante, odometría, estado compartido protegido con mutex. `server.c` es una versión anterior/más simple. Ambos son programas reales y específicos del proyecto, no ejemplos del proveedor, pese al nombre de fichero genérico heredado de la propia plantilla de `libkhepera`.

Compilar con:
```
make LIBKHEPERA_PATH=/ruta/a/libkhepera-2.1/build-khepera-<version-kernel>
```
Ver el comentario al principio del `Makefile` para compilación cruzada hacia el objetivo ARM real del Khepera. Verificado de forma real en esta sesión: ambos ficheros compilan limpio contra las cabeceras de un `libkhepera-2.1` real (solo avisos, sin errores) — el enlazado final contra la librería compilada no se probó aquí (esa compilación es solo ARM, este sandbox es x86_64).

## Arduino / ESP32 (`Arduino/`, `arduino-khepera-uROS/`, `ESP32-SerialCOM/`)

Prototipos micro-ROS. Cada sketch con WiFi lee su SSID/contraseña de un `arduino_secrets.h` local, ignorado por git (copia el `.h.example` de al lado y rellena tus propias credenciales — nunca subas el fichero real). **Pendiente de revisión manual** (ver `AUDIT.md`, rama `doc`) — no tocado en esta pasada, hay que decidir cuáles siguen siendo útiles.

## Matlab (`Matlab/`)

`KheperaIV_Launch.m`, `KheperaIV_Model.m` + su `.slx`. **Pendiente de revisión manual** (ver `AUDIT.md`, rama `doc`) — no tocado en esta pasada.
