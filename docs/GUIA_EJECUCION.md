# Guía de ejecución — Workshop 3 (Grupo 14: R A S C)

Todo lo que hay que hacer para reproducir el trabajo, en orden. Los comandos se
ejecutan en Ubuntu 24.04 con ROS 2 Jazzy y Gazebo Harmonic.

> **Cada terminal nueva** necesita:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/workshop_03/install/setup.bash
> ```

---

## 0. Compilar (una sola vez)

```bash
cd ~/workshop_03
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

Si cambias un `.py`, un `.launch.py` o un `.rviz`, **no hace falta recompilar**
(`--symlink-install`). Si cambias `setup.py` o `package.xml`, sí:

```bash
colcon build --symlink-install --packages-select omni_workshop
```

---

## 1. Verificar el modelo sin Gazebo (rápido)

```bash
cd ~/workshop_03/src/omni_workshop

# Cinemática: IK -> FK, casos conocidos, saturación
python3 tools/test_kinematics.py

# Figuras: tamaño, continuidad, cierre. Guarda docs/img/figures_check.png
python3 tools/validate_figures.py grupo14 RASC

# Plano de los mundos (para entender el mapeo)
python3 tools/plot_world.py office_0 office_4
```

---

## 2. Parte 1 — figura propia (`grupo14`)

**Terminal 1** (deja Gazebo y RViz abiertos):

```bash
ros2 launch omni_workshop workshop.launch.py rviz:=true
```

Espera el mensaje **`Wheel controller active`**.

**Terminal 2**:

```bash
ros2 run omni_workshop self_controller --ros-args -p figure:=grupo14
```

Al terminar, el `plotter` guarda automáticamente:

```
~/workshop_results/trajectory_YYYYmmdd_HHMMSS.png
```

Ese PNG trae las tres curvas:

| Color | Qué es |
|---|---|
| negro discontinuo | trayectoria **deseada** (referencia) |
| azul | **pose real** en Gazebo (verdad de terreno) |
| naranja | **odometría** calculada con las 4 ruedas |

y un segundo panel con el error instantáneo y el RMS de cada una.

Para repetir la figura hay que relanzar la Terminal 1 (el robot no vuelve solo
al punto de partida).

---

## 3. Parte 1 — iniciales `R A S C`

La palabra mide 2.65 × 1.10 m, así que el robot debe arrancar **desplazado** para
que no choque con las paredes de la arena de 6 × 6 m.

**Terminal 1**:

```bash
ros2 launch omni_workshop workshop.launch.py rviz:=true x:=-1.3 y:=-0.8
```

**Terminal 2**:

```bash
ros2 run omni_workshop self_controller --ros-args -p figure:=RASC
```

Dura ~90 s. El robot se mantiene mirando hacia +x todo el tiempo y solo se
traslada (ventaja holonómica). Al final, otro PNG en `~/workshop_results/`.

> Si quieres ver la palabra "derecha" en la pantalla de RViz, gira la vista
> superior (tecla `T` en RViz y luego arrastra) o simplemente abre el PNG, que
> ya viene con los ejes correctos.

---

## 4. Parte 2 — mapeo con el controlador de fábrica

### 4.0 Si el computador va lento (RTF bajo)

Gazebo corre más lento que la realidad cuando no alcanza la CPU. En un equipo de
4 núcleos sin GPU el robot de fábrica (dos LiDAR Hokuyo a 40 Hz y 0.5°) da un
**RTF ≈ 0.15**: 1 s de simulación tarda ~7 s reales. Se midió lo siguiente:

| Configuración | RTF |
|---|---|
| `dingo_o.yaml` (2 LiDAR, 40 Hz, 0.5°) — de fábrica | 0.15 |
| `robot_config:=dingo_o_fast.yaml` (1 LiDAR, 40 Hz, 0.5°) | 0.25 |
| `robot_config:=dingo_o_fast.yaml lidar_rate:=20 lidar_ang_res:=1.0` | **0.48** |
| sin LiDAR (solo para medir) | 0.63 |

El segundo LiDAR es redundante para SLAM (los dos son de 360°); el primero basta.
Bajar a 20 Hz y 1° cuesta 4 veces menos y el mapa sale igual, porque
`slam_toolbox` ya descarta escaneos (`throttle_scans: 6`).

**Recomendado en equipos lentos:**

```bash
ros2 launch omni_workshop simulation.launch.py world:=office_0 \
  robot_config:=dingo_o_fast.yaml lidar_rate:=20 lidar_ang_res:=1.0
```

Los argumentos `lidar_rate` y `lidar_ang_res` regeneran la descripción del robot
y reescriben la frecuencia y la resolución del LiDAR antes de arrancar Gazebo
(el generador de Clearpath las tiene fijas en 40 Hz y 0.5°). Con los valores por
defecto no se toca nada.

> En la Parte 1 los LiDAR no se usan: se puede usar la misma configuración rápida
> para que la figura termine antes.

### 4.1 Entorno pequeño: `office_0` (11 × 9.75 m)

**Terminal 1**:

```bash
ros2 launch omni_workshop simulation.launch.py world:=office_0
```

**Terminal 2**:

```bash
ros2 launch omni_workshop mapping.launch.py
```

**Terminal 3** (conducir): usa el panel **Omnidirectional teleop** de la ventana
de Gazebo (columna derecha). Con el panel enfocado también responde a
`W/S` (adelante/atrás), `A/D` (lateral), `Q/E` (giro).

**Terminal 4** (guardar cuando el mapa esté completo):

```bash
mkdir -p ~/workshop_results/maps
ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \
  --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
```

Genera `office_0.pgm` + `office_0.yaml`.

### 4.2 Entorno grande: `office_4` (32 × 18.15 m, clínica)

Igual, cambiando el mundo:

```bash
ros2 launch omni_workshop simulation.launch.py world:=office_4
ros2 launch omni_workshop mapping.launch.py
# ... conducir ...
ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_4 \
  --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
```

`office_4` es pesado para 4 núcleos. Si Gazebo va muy lento, usa `office_1`
(19.3 × 10.65 m), que también es "entorno grande" y corre más fluido.

### 4.3 Exploración automática (opcional, para generar el mapa sin conducir)

```bash
ros2 run omni_workshop explore --ros-args -p time_limit:=600.0
```

Elige metas en la frontera del mapa de SLAM y se detiene solo con
`EXPLORATION FINISHED`. Después guarda el mapa con el comando de arriba.

---

## 5. Dónde queda cada cosa

```
~/workshop_03/
└── src/
    ├── omni_workshop/omni_workshop/
    │   ├── self_controller.py   # cinemática mecanum propia
    │   ├── trajectory.py        # figuras grupo14 y RASC
    │   ├── plotter.py           # 3 poses + RMS
    │   └── explore.py           # exploración automática (bonus)
    ├── omni_workshop/tools/
    │   ├── test_kinematics.py
    │   ├── validate_figures.py
    │   └── plot_world.py
    ├── docs/
    │   ├── INFORME_WS3.md       # entregable escrito
    │   ├── GUIA_EJECUCION.md    # este archivo
    │   ├── GUION_VIDEO.md
    │   └── img/
    ├── maps/                    # .pgm + .yaml de los dos entornos
    ├── scripts/empaquetar.sh
    └── README_WS3.md
```

Los resultados de cada corrida (PNG de trayectorias) van a
`~/workshop_results/`.

---

## 6. Problemas comunes

| Síntoma | Solución |
|---|---|
| `Package 'omni_workshop' not found` | falta `source install/setup.bash` |
| `self_controller` se queda en *Waiting for the simulation* | Gazebo no está corriendo o aún no salió `Wheel controller active` |
| El robot no responde / *No clock received* | quedó una simulación vieja: `pkill -f "gz sim"` y relanzar |
| El panel de teleop no mueve el robot | lanzaste `workshop.launch.py` (controlador propio); para mapear usa `simulation.launch.py` |
| RViz no encuentra `map` o el robot | abre RViz con `mapping.launch.py` (remapea `/tf` al namespace) |
| Error `GLSL link result ... indexed_8bit_image` | aviso del driver al dibujar el mapa; si el mapa se ve, ignóralo |
| El robot choca | no hay espacio desde donde quedó; relanza la simulación |
| Compilación congelada por RAM | `MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2` |
