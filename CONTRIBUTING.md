# Contribuir a este repositorio

Guía común para los repositorios de [Robotic Park Lab](https://github.com/Robotic-Park-Lab). Si eres nuevo en el laboratorio, empieza por [RoboticPark](https://github.com/Robotic-Park-Lab/RoboticPark) y su `install.sh`, que instala el workspace completo con el resto de paquetes del laboratorio.

## Ramas

- `humble-dev` (o la rama de desarrollo activo que indique el README de cada repo): sobre ROS 2 Humble, es la rama estable de referencia del laboratorio.
- `ros2-foxy`, `ros2-galactic`, `ros-noetic`, etc.: ramas de compatibilidad para quien trabaje con una distro anterior. Se aceptan mejoras puntuales, pero no tienen el mismo ritmo de mantenimiento que la rama de desarrollo activo.
- `benchmark`: **no se modifica ni se renombra**. Respalda una publicación del laboratorio (capítulo de libro sobre control) y su historia debe permanecer intacta. Cualquier actualización se reproduce reinstalando desde `RoboticPark/install.sh`, nunca con push directo a esta rama.

## Antes de abrir un Pull Request

1. Comprueba que el paquete compila (`colcon build --packages-select <paquete>`) y, si aplica, que pasa `colcon test`.
2. Sigue la convención de nombres ya usada en el repo (`uned_<paquete>_<rol>`, p. ej. `_config`, `_driver`, `_task`, `_gui`, `_webots`).
3. Actualiza el README si cambias la estructura de paquetes, las dependencias o la forma de uso.
4. Si el cambio está ligado a un ejercicio, proyecto o publicación del laboratorio, indícalo en la descripción del PR.

## Estilo de código

- Python (`ament_python`): `ament_flake8` / `ament_pep257` — revisa `test/` en cada paquete.
- C++ (`ament_cmake`): sigue el estilo ya presente en el paquete (`-Wall -Wextra -Wpedantic`).

## Licencia

El código de este repositorio se publica bajo licencia BSD 3-Clause (ver `LICENSE`). Al contribuir, aceptas que tu aportación se publique bajo la misma licencia.
