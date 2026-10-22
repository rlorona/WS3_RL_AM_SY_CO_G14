# Robot omnidireccional en el laboratorio CoRAL

Simulación del **Clearpath Dingo-O** (4 ruedas mecanum) dentro de una réplica del laboratorio CoRAL, y el material del taller de modelo cinemático y trayectorias.

**Probado en:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic (gz-sim 8)

## Contenido de `src/`

| Carpeta | Qué es |
|---|---|
| `coral_world/` | Mundo de Gazebo del laboratorio (generado desde el plano PL-002), configuración del robot y launch de la simulación |
| `omni_teleop_gui/` | Panel de la GUI de Gazebo para teleoperar en las 8 direcciones y girar |
| `taller_omni/` | Taller: nodos, plantilla para los alumnos y notebook. Ver [`taller_omni/README.md`](taller_omni/README.md) |
| `clearpath_ws/src/` | Repositorios oficiales de Clearpath (simulador, descripción, controladores). No se editan |
| `dependencias.repos` | Lista de los repositorios de Clearpath, en la versión probada |

## Instalación

> Guía completa, con la descarga y compilación de Clearpath y los problemas frecuentes: [`REQUERIMIENTOS.md`](REQUERIMIENTOS.md).

Requisitos previos: ROS 2 Jazzy y Gazebo Harmonic (`sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz`), además de `colcon`, `vcstool` y `rosdep`.

```bash
# 1. Workspace con esta carpeta como src/
mkdir -p ~/practica_03 && cd ~/practica_03
#    (copiar o clonar aquí la carpeta src/)

# 2. Repositorios de Clearpath (si no están ya en src/clearpath_ws/)
cd ~/practica_03/src
vcs import < dependencias.repos

# 3. Dependencias del sistema (pide la contraseña de sudo)
cd ~/practica_03
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot

# 4. Compilar
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` hace que los cambios en archivos Python, launch y del mundo se vean sin recompilar.

Para abrir el notebook del taller hace falta Jupyter: `sudo apt install jupyter-notebook`. También se puede abrir en VS Code con la extensión de Jupyter.

## Simulación

```bash
source ~/practica_03/install/setup.bash
ros2 launch coral_world dingo_coral.launch.py            # agrega rviz:=true para abrir RViz
```

En la GUI de Gazebo, a la derecha:

- **Teleop omnidireccional**: mantener presionada una flecha mueve el robot y al soltar se detiene. Con el panel enfocado también responde a `W`/`S` (avanzar), `A`/`D` (lateral) y `Q`/`E` (girar).
- **Lidar delantero / trasero**: pulsar ↻ y elegir el topic `/do100_0000/sensors/lidar2d_X/scan` para ver los rayos.

Teleoperar desde la terminal (con Shift + `J`/`L` se mueve de lado):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/do100_0000/cmd_vel -p stamped:=true -p frame_id:=base_link -p use_sim_time:=true
```

| Topic | Contenido |
|---|---|
| `/do100_0000/cmd_vel` | comando de velocidad (`TwistStamped`) |
| `/do100_0000/platform/odom/filtered` | odometría (EKF) |
| `/do100_0000/sensors/lidar2d_0/scan`, `lidar2d_1/scan` | lidars delantero y trasero |
| `/do100_0000/tf` | TF del robot (namespaced; el frame fijo es `odom`) |

## Notas

- **El robot es el mismo para todos:** está definido en `coral_world/config/dingo_o.yaml`. Al lanzar, Clearpath genera a partir de él el URDF, los parámetros y los launch en `~/.ros/coral_world/clearpath/`. Esos archivos se regeneran en cada lanzamiento y no hace falta tocarlos.
- **Cambiar el mundo:** edita `coral_world/scripts/generar_mundo.py` (medidas, colores, tamaño de baldosa), ejecútalo y vuelve a lanzar. Genera `worlds/coral.sdf` y la textura del piso.
- **Origen del mundo:** el centro del laboratorio, con +x a la derecha del plano y +y hacia arriba.
