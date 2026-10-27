# Requerimientos del workshop 03

## 1. Ubuntu 24.04 y ROS 2 Jazzy

- Ubuntu 24.04 LTS: [descarga e instalación](https://ubuntu.com/download/desktop)
- ROS 2 Jazzy: [instalación en Ubuntu (paquetes deb)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 2. Source de ROS 2

Cada terminal nueva necesita cargar el entorno de ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

Para que se cargue solo en cada terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 3. Clonar este repositorio y descargar los repositorios de Clearpath

El repositorio es la carpeta `src/` del workspace:

```bash
mkdir -p ~/workshop_03
git clone https://github.com/davatorr1009/workshop_03.git ~/workshop_03/src
```

Los repositorios de Clearpath **no** vienen en este repositorio: cada estudiante los descarga dentro de su workspace con `vcs`, a partir de `dependencias.repos` (fija cada repositorio en el commit de la rama `jazzy` con el que se probó el taller).

```bash
sudo apt install python3-vcstool
cd ~/workshop_03/src
vcs import < dependencias.repos
```

Esto crea `src/clearpath_ws/src/` con 6 repositorios (~1 GB): `clearpath_simulator`, `clearpath_common`, `clearpath_config`, `clearpath_msgs`, `clearpath_desktop` y `clearpath_nav2_demos` (configuración de navegación y mapeo para el Dingo-O, ver la sección 4). Para comprobar que quedaron en la versión correcta:

```bash
vcs log -l 1 clearpath_ws/src      # cada uno debe mostrar el commit de dependencias.repos
```

> Si `vcs import` falla a la mitad (por ejemplo por la red), se puede volver a ejecutar: salta lo que ya está descargado.

## 4. Navegación y mapeo

Paquetes de Nav2 (navegación) y slam_toolbox (mapeo):

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

La configuración de estos paquetes para el Dingo-O viene en `clearpath_nav2_demos`, que ya se descargó en la sección 3 y se compila con el resto del workspace.

## 5. Dependencias y compilación

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions python3-rosdep
cd ~/workshop_03
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot
colcon build --symlink-install
source install/setup.bash
```

- `--symlink-install` hace que los cambios en archivos Python, launch y mundos se vean sin recompilar.
- `--skip-keys clearpath_generator_robot` es necesario porque ese paquete solo existe en los robots reales de Clearpath.
- Si es la primera vez que se usa `rosdep` en el equipo, antes hace falta `sudo rosdep init`.
- Si el equipo se congela compilando (poca RAM): `MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2`.
- Si `apt` responde *"Could not get lock /var/lib/dpkg/lock-frontend ... held by process (unattended-upgr)"*, Ubuntu está instalando actualizaciones automáticas: espera a que terminen (pueden tardar varios minutos) y vuelve a ejecutar el comando. No borres el archivo de bloqueo ni mates el proceso.

Con esto listo, sigue el taller en el [`README.md`](README.md).
