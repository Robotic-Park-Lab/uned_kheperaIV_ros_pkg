# Documentación de ramas y guía de contribución — uned_kheperaIV_ros_pkg

Esta rama (`doc`) no contiene código: solo explica para qué sirve cada rama del repositorio y cómo contribuir.

📋 **[AUDIT.md](AUDIT.md)** — checklist de la auditoría de `humble-dev` (2026-08-22), de lo más simple a lo más complejo. Se va tachando conforme se resuelve.

💡 **[IDEAS_FUTURAS.md](IDEAS_FUTURAS.md)** — documento vivo para ideas y trabajo futuro que no son bugs pendientes de la auditoría.

## Ramas de este repositorio

| Rama | Propósito |
|---|---|
| `humble-dev` | Desarrollo activo, sobre ROS 2 Humble. Rama por defecto del repo. |
| `benchmark` | **No se modifica ni se renombra.** Bloqueada en GitHub (`lock_branch`) incluso para administradores. Respalda un capítulo de libro sobre control publicado por Francisco Mañas. Cualquier actualización se reproduce reinstalando desde `RoboticPark/install.sh`, nunca con push directo. |
| `doc` (esta) | Documentación de ramas y guía de contribución, común a todo el laboratorio. |

## Guía de contribución

Antes de abrir un Pull Request:
1. Comprueba que el paquete compila (`colcon build --packages-select <paquete>`) y, si aplica, que pasa `colcon test`.
2. Sigue la convención de nombres ya usada en el repo (`uned_<paquete>_<rol>`, p. ej. `_config`, `_driver`, `_task`, `_gui`, `_webots`).
3. Actualiza el README de la rama de desarrollo si cambias estructura, dependencias o forma de uso — y esta página si cambias el propósito de una rama o añades una nueva.
4. Si el cambio está ligado a un ejercicio, proyecto o publicación del laboratorio, indícalo en la descripción del PR.

### Estilo de código
- Python (`ament_python`): `ament_flake8` / `ament_pep257`.
- C++ (`ament_cmake`): sigue el estilo ya presente en el paquete (`-Wall -Wextra -Wpedantic`).

### Licencia
El código de este repositorio se publica bajo licencia BSD 3-Clause (ver `LICENSE` en `humble-dev`). Al contribuir, aceptas que tu aportación se publique bajo la misma licencia.
