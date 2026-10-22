# Requerimientos e instalación del workshop 03

Guía paso a paso para dejar funcionando la simulación del Clearpath Dingo-O en el laboratorio CoRAL. Lo más delicado es **descargar y compilar los paquetes de Clearpath desde el código fuente**, porque no vienen en este repositorio.

## 1. Sistema

| Requisito | Versión probada |
|---|---|
| Sistema operativo | Ubuntu 24.04 LTS (nativo o en máquina virtual; en Windows también sirve WSL2 con Ubuntu 24.04) |
| ROS 2 | Jazzy Jalisco |
| Gazebo | Harmonic (gz-sim 8) |
| RAM | 8 GB como mínimo. Con menos, la compilación puede quedarse sin memoria (ver la nota del paso 5) |
| Disco | ~3 GB libres para fuentes, `build/` e `install/` |

## 2. Paquetes del sistema

Con ROS 2 Jazzy ya instalado según la [guía oficial](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html):

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
                 python3-colcon-common-extensions python3-vcstool python3-rosdep git
```

Si nunca se usó `rosdep` en el equipo:

```bash
sudo rosdep init
rosdep update
```

## 3. Clonar este repositorio

El repositorio es la carpeta `src/` del workspace. El notebook y los README usan la ruta `~/practica_03`, así que conviene clonarlo exactamente ahí:

```bash
mkdir -p ~/practica_03
git clone https://github.com/davatorr1009/workshop_03.git ~/practica_03/src
```

## 4. Descargar los paquetes de Clearpath

Los repositorios oficiales de Clearpath (simulador, descripción del robot, controladores y mensajes) **no** se suben a este repositorio, porque pesan cerca de 1 GB y no se editan. Se descargan con `vcs` a partir de `dependencias.repos`, que fija cada repositorio en el commit de la rama `jazzy` con el que se probó el taller:

```bash
cd ~/practica_03/src
vcs import < dependencias.repos
```

Esto crea `src/clearpath_ws/src/` con 5 repositorios:

| Repositorio | Para qué se usa |
|---|---|
| `clearpath_simulator` | Integración con Gazebo (`clearpath_gz`) y generador de la simulación |
| `clearpath_common` | Descripción URDF de la plataforma, control y generadores |
| `clearpath_config` | Lectura del `robot.yaml` que define el robot |
| `clearpath_msgs` | Mensajes propios de Clearpath |
| `clearpath_desktop` | Visualización en RViz |

Para comprobar que quedaron en la versión correcta:

```bash
vcs log -l 1 clearpath_ws/src      # cada uno debe mostrar el commit de dependencias.repos
```

> Si `vcs import` falla a la mitad (por ejemplo por la red), se puede volver a ejecutar: salta lo que ya está descargado.

## 5. Instalar dependencias y compilar

```bash
cd ~/practica_03
source /opt/ros/jazzy/setup.bash

# dependencias de todos los paquetes (pide la contraseña de sudo)
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot

# compilar todo el workspace: Clearpath y los paquetes del taller
colcon build --symlink-install
```

- `--skip-keys clearpath_generator_robot` es necesario porque ese paquete solo existe en los robots reales de Clearpath, y sin esta opción `rosdep` se detiene con un error.
- La primera compilación tarda varios minutos. Las siguientes son rápidas.
- Si el equipo tiene poca RAM y se congela, compila con menos procesos en paralelo:
  ```bash
  MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
  ```
- Si después solo cambias los paquetes del taller, no hace falta recompilar Clearpath:
  ```bash
  colcon build --symlink-install --packages-select coral_world taller_omni omni_teleop_gui
  ```

## 6. Probar que funciona

```bash
cd ~/practica_03
source install/setup.bash
ros2 launch coral_world dingo_coral.launch.py
```

Debe abrirse Gazebo con el laboratorio CoRAL y el Dingo-O cerca de la entrada. La GUI tarda unos segundos más que el servidor, porque espera a que el robot publique sus lidars.

Para no tener que hacer `source` en cada terminal:

```bash
echo "source ~/practica_03/install/setup.bash" >> ~/.bashrc
```

## Problemas frecuentes

| Síntoma | Causa y solución |
|---|---|
| `Package 'clearpath_gz' not found` | Faltó el paso 4, o no se hizo `source install/setup.bash` después de compilar |
| `rosdep` falla con `clearpath_generator_robot` | Falta `--skip-keys clearpath_generator_robot` |
| La compilación se congela o se corta | Falta RAM: usa `--parallel-workers 2` (paso 5) |
| Gazebo abre pero sin el robot, o el siguiente lanzamiento falla | Quedó un servidor de una ejecución anterior: `pkill -f "gz sim"` y volver a lanzar |
| El robot no aparece en una máquina virtual | Activa la aceleración 3D de la máquina virtual o lanza con `LIBGL_ALWAYS_SOFTWARE=1` |

Para el uso de la simulación y el taller, ver [`README.md`](README.md) y [`taller_omni/README.md`](taller_omni/README.md).
