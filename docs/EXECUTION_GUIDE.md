# Execution guide — Workshop 3 (Team 14: R A S C)

Everything you need to reproduce the work, in order. Commands run on
Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic.

> **Every new terminal** needs:
> ```bash
> source /opt/ros/jazzy/setup.bash
> source ~/workshop_03/install/setup.bash
> ```

---

## 0. Build (once)

```bash
cd ~/workshop_03
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

If you change a `.py`, a `.launch.py` or an `.rviz` file, **no rebuild is
needed** (`--symlink-install`). If you change `setup.py` or `package.xml`, it
is:

```bash
colcon build --symlink-install --packages-select omni_workshop
```

---

## 1. Verify the model without Gazebo (fast)

```bash
cd ~/workshop_03/src/omni_workshop

# Kinematics: IK -> FK, known cases, saturation
python3 tools/test_kinematics.py

# Figures: size, continuity, closing (tools/validate_figures.py)
python3 tools/validate_figures.py grupo14 RASC

# Floor plan of the worlds (to understand the mapping)
python3 tools/plot_world.py office_0 office_4
```

---

## 2. Part 1 — team figure (`grupo14`)

**Terminal 1** (leave Gazebo and RViz open):

```bash
ros2 launch omni_workshop workshop.launch.py rviz:=true
```

Wait for the **`Wheel controller active`** message.

**Terminal 2**:

```bash
ros2 run omni_workshop self_controller --ros-args -p figure:=grupo14
```

When it finishes, the `plotter` saves automatically:

```
~/workshop_results/trajectory_YYYYmmdd_HHMMSS.png
```

That PNG carries the three curves:

| Color | What it is |
|---|---|
| dashed black | **desired** trajectory (reference) |
| blue | **real pose** in Gazebo (ground truth) |
| orange | **odometry** computed from the 4 wheels |

and a second panel with the instantaneous error and the RMS of each one.

To repeat the figure, relaunch Terminal 1 (the robot does not return to the
start point by itself).

---

## 3. Part 1 — initials `R A S C`

The word measures 2.65 × 1.10 m, so the robot must start **offset** to avoid
crashing into the walls of the 6 × 6 m arena.

**Terminal 1**:

```bash
ros2 launch omni_workshop workshop.launch.py rviz:=true x:=-1.3 y:=-0.8
```

**Terminal 2**:

```bash
ros2 run omni_workshop self_controller --ros-args -p figure:=RASC
```

It takes ~90 s. The robot keeps facing +x the whole time and only translates
(holonomic advantage). At the end, another PNG lands in `~/workshop_results/`.

> To see the word "right side up" in RViz, rotate the top view (press `T` in
> RViz and drag) or simply open the PNG, which already has the correct axes.

---

## 4. Part 2 — mapping with the factory controller

### 4.0 If the computer is slow (low RTF)

Gazebo runs slower than real time when the CPU is not enough. On a 4-core
machine without GPU the factory robot (two Hokuyo LiDARs at 40 Hz and 0.5°)
gives **RTF ≈ 0.15**: 1 s of simulation takes ~7 real seconds. Measured values:

| Configuration | RTF |
|---|---|
| `dingo_o.yaml` (2 LiDARs, 40 Hz, 0.5°) — factory | 0.15 |
| `robot_config:=dingo_o_fast.yaml` (1 LiDAR, 40 Hz, 0.5°) | 0.25 |
| `robot_config:=dingo_o_fast.yaml lidar_rate:=20 lidar_ang_res:=1.0` | **0.48** |
| without LiDAR (only to measure) | 0.63 |

The second LiDAR is redundant for SLAM (both are 360°); the first one is
enough. Dropping to 20 Hz and 1° costs 4× less and the map comes out the same,
because `slam_toolbox` already throttles scans (`throttle_scans: 6`).

**Recommended on slow machines:**

```bash
ros2 launch omni_workshop simulation.launch.py world:=office_0 \
  robot_config:=dingo_o_fast.yaml lidar_rate:=20 lidar_ang_res:=1.0
```

The `lidar_rate` and `lidar_ang_res` arguments regenerate the robot
description and rewrite the LiDAR rate and resolution before starting Gazebo
(the Clearpath generator hardcodes 40 Hz and 0.5°). With the default values,
nothing is touched.

> In Part 1 the LiDARs are not used: the same fast configuration works so the
> figure finishes sooner.

### 4.1 Small environment: `office_0` (11 × 9.75 m)

**Terminal 1**:

```bash
ros2 launch omni_workshop simulation.launch.py world:=office_0
```

**Terminal 2**:

```bash
ros2 launch omni_workshop mapping.launch.py
```

**Terminal 3** (drive): use the **Omnidirectional teleop** panel of the Gazebo
window (right column). With the panel focused it also responds to `W/S`
(forward/backwards), `A/D` (lateral), `Q/E` (rotation).

**Terminal 4** (save once the map is complete):

```bash
mkdir -p ~/workshop_results/maps
ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \
  --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
```

It produces `office_0.pgm` + `office_0.yaml`.

### 4.2 Large environment: `office_4` (32 × 18.15 m, clinic)

Same, changing the world:

```bash
ros2 launch omni_workshop simulation.launch.py world:=office_4
ros2 launch omni_workshop mapping.launch.py
# ... drive ...
ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_4 \
  --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
```

`office_4` is heavy for 4 cores. If Gazebo is too slow, use `office_1`
(19.3 × 10.65 m), which also counts as a "large environment" and runs smoother.

### 4.3 Automatic exploration (optional, to build the map without driving)

```bash
ros2 run omni_workshop explore --ros-args -p time_limit:=600.0
```

It picks goals on the frontier of the SLAM map and stops by itself with
`EXPLORATION FINISHED`. Then save the map with the command above.

> If the robot escapes through an exterior door (the `office_*` worlds have
> openings to the outer plane and the LiDAR reaches 20–25 m through them),
> use the `coverage.py` alternative, which follows the internal floor plan:
>
> ```bash
> ros2 run omni_workshop coverage --ros-args -p world:=office_0 -p robot_radius:=0.45
> ```
>
> and save the map when it prints `COVERAGE FINISHED`.

---

## 5. Where everything lands

```
~/workshop_03/
└── src/
    ├── omni_workshop/omni_workshop/
    │   ├── self_controller.py   # own mecanum kinematics
    │   ├── trajectory.py        # figures grupo14 and RASC
    │   ├── plotter.py           # 3 poses + RMS
    │   └── explore.py           # automatic exploration (bonus)
    ├── omni_workshop/tools/
    │   ├── test_kinematics.py
    │   ├── validate_figures.py
    │   └── plot_world.py
    ├── docs/
    │   ├── REPORT_WS3.md       # written deliverable
    │   ├── EXECUTION_GUIDE.md  # this file
    │   └── img/
    ├── maps/                    # .pgm + .yaml of both environments
    ├── scripts/empaquetar.sh
    └── README_WS3.md
```

The results of each run (trajectory PNGs) go to `~/workshop_results/`.

---

## 6. Common problems

| Symptom | Solution |
|---|---|
| `Package 'omni_workshop' not found` | missing `source install/setup.bash` |
| `self_controller` stays at *Waiting for the simulation* | Gazebo is not running, or `Wheel controller active` has not appeared yet |
| The robot does not respond / *No clock received* | an old simulation is left over: `pkill -f "gz sim"` and relaunch |
| The teleop panel does not move the robot | you launched `workshop.launch.py` (own controller); for mapping use `simulation.launch.py` |
| RViz cannot find `map` or the robot | open RViz with `mapping.launch.py` (it remaps `/tf` to the namespace) |
| Error `GLSL link result ... indexed_8bit_image` | driver warning while drawing the map; if the map looks fine, ignore it |
| The robot crashes | no space left where it stopped; relaunch the simulation |
| Build frozen due to RAM | `MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2` |
