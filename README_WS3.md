# WS3 — Grupo 14 (R A S C)

Workshop 3: controlador y odometría propios para el **Clearpath Dingo-O**
(4 ruedas mecanum a 45°) y mapeo con el controlador de fábrica.

- **Curso:** Robótica / ROS 2
- **Integrantes:** Ricardo (R), Alexa (A), Steven (S), Cristhian (C)
- **Stack:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic 8.15

## Inicio rápido

```bash
cd ~/workshop_03
source /opt/ros/jazzy/setup.bash
vcs import < src/dependencies.repos      # solo la primera vez (~1 GB)
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

### Parte 1 — figura e iniciales

```bash
# terminal 1
ros2 launch omni_workshop workshop.launch.py rviz:=true x:=-1.3 y:=-0.8
# terminal 2
ros2 run omni_workshop self_controller --ros-args -p figure:=grupo14
ros2 run omni_workshop self_controller --ros-args -p figure:=RASC
```

Los PNG con las tres curvas (deseada, real, odometría) quedan en
`~/workshop_results/`.

### Parte 2 — mapeo

```bash
# terminal 1
ros2 launch omni_workshop simulation.launch.py world:=office_0   # u office_4
# terminal 2
ros2 launch omni_workshop mapping.launch.py
# terminal 3, al terminar de recorrer
ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \
  --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
```

## Documentación

| Archivo | Contenido |
|---|---|
| [`docs/GUIA_EJECUCION.md`](docs/GUIA_EJECUCION.md) | paso a paso de todo, con solución de problemas |
| [`docs/INFORME_WS3.md`](docs/INFORME_WS3.md) | informe con las respuestas 2.1–2.5 |
| [`docs/GUION_VIDEO.md`](docs/GUION_VIDEO.md) | guion de las tomas para el video |
| `maps/` | mapas `.pgm` + `.yaml` de los dos entornos |
| `docs/img/` | figuras de validación y resultados |

## Estructura

```
src/omni_workshop/
├── omni_workshop/
│   ├── self_controller.py   # cinemática mecanum propia
│   ├── trajectory.py        # figuras (grupo14, RASC, ...)
│   ├── plotter.py           # comparación de las 3 poses + RMS
│   └── explore.py           # exploración automática (bonus)
├── tools/
│   ├── test_kinematics.py   # IK/FK, casos conocidos, saturación
│   ├── validate_figures.py  # tamaño, continuidad y cierre de las figuras
│   └── plot_world.py        # plano de los mundos desde el SDF
├── launch/                  # workshop, simulation, mapping
├── config/                  # robot, controlador de ruedas, mundos
└── rviz/
```

## Validación

```bash
python3 src/omni_workshop/tools/test_kinematics.py
python3 src/omni_workshop/tools/validate_figures.py grupo14 RASC
```

## Licencia

El material base del workshop es de David Torres (Apache-2.0). Las
modificaciones de este repositorio son del Grupo 14.
