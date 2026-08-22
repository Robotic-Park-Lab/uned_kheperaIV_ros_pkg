# Scripts

Auxiliary files that are not part of any ROS 2 package.

## Onboard Khepera IV programs (`server.c`, `prog-template.c`)

C programs that run on the Khepera IV's own onboard embedded Linux, built against [`libkhepera`](https://www.k-team.com/) (K-Team SA's Khepera SDK — **not vendored in this repo**, see the root README's "Installation" section and `AUDIT.md` on the `doc` branch for why). They open a TCP server (port 50000) that `uned_kheperaiv_driver`'s `kheperaIV_client_driver` node connects to as a client.

`prog-template.c` ("Código optimizado para Khepera IV - Comunicación ROS2", by Francisco José Mañas Álvarez, 05-2025) is the newer, more complete version — motor control with smooth interpolation, non-blocking TCP, odometry, mutex-protected shared state. `server.c` is an earlier/simpler version. Both are real, project-specific programs, not vendor examples, despite the generic filename inherited from `libkhepera`'s own template.

Build with:
```
make LIBKHEPERA_PATH=/path/to/libkhepera-2.1/build-khepera-<kernel-version>
```
See the comment at the top of `Makefile` for cross-compiling to the Khepera's actual ARM target. Verified for real in this session: both files compile cleanly against a real `libkhepera-2.1` checkout's headers (warnings only, no errors) — final linking against the compiled library wasn't tested here (that build is ARM-only, this sandbox is x86_64).

## Arduino / ESP32 (`Arduino/`, `arduino-khepera-uROS/`, `ESP32-SerialCOM/`)

micro-ROS prototypes. Each WiFi-enabled sketch reads its SSID/password from a local, gitignored `arduino_secrets.h` (copy the `.h.example` next to it and fill in your own credentials — never commit the real file). **Pending manual review** (see `AUDIT.md`, `doc` branch) — not otherwise touched in this pass, decide which are still useful.

## Matlab (`Matlab/`)

`KheperaIV_Launch.m`, `KheperaIV_Model.m` + its `.slx`. **Pending manual review** (see `AUDIT.md`, `doc` branch) — not touched in this pass.
