# WS3 — Team 14 (R A S C)

Workshop 3: own controller and odometry for the **Clearpath Dingo-O** (4
mecanum wheels at 45°) plus mapping with the factory controller.

- **Course:** Robotics / ROS 2
- **Members:** Ricardo (R), Alexa (A), Steven (S), Cristhian (C)
- **Stack:** Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic 8.15

## Quick start

```bash
cd ~/workshop_03
source /opt/ros/jazzy/setup.bash
vcs import < src/dependencies.repos      # first time only (~1 GB)
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot
MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

### Part 1 — figure and initials

```bash
# terminal 1
ros2 launch omni_workshop workshop.launch.py rviz:=true x:=-1.3 y:=-0.8
# terminal 2
ros2 run omni_workshop self_controller --ros-args -p figure:=grupo14
ros2 run omni_workshop self_controller --ros-args -p figure:=RASC
```

The PNGs with the three curves (desired, real, odometry) land in
`~/workshop_results/`.

### Part 2 — mapping

```bash
# terminal 1
ros2 launch omni_workshop simulation.launch.py world:=office_0   # or office_4
# terminal 2
ros2 launch omni_workshop mapping.launch.py
# terminal 3, once the environment has been covered
ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \
  --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
```

## Documentation

| File | Content |
|---|---|
| [`docs/EXECUTION_GUIDE.md`](docs/EXECUTION_GUIDE.md) | step-by-step for everything, with troubleshooting |
| [`docs/REPORT_WS3.md`](docs/REPORT_WS3.md) | report with answers 2.1–2.5 |
| `maps/` | `.pgm` + `.yaml` maps of both environments |
| `docs/img/` | validation and result figures |

## Structure

```
src/omni_workshop/
├── omni_workshop/
│   ├── self_controller.py   # own mecanum kinematics
│   ├── trajectory.py        # figures (grupo14, RASC, ...)
│   ├── plotter.py           # comparison of the 3 poses + RMS
│   └── explore.py           # automatic exploration (bonus)
├── tools/
│   ├── test_kinematics.py   # IK/FK, known cases, saturation
│   ├── validate_figures.py  # size, continuity and closing of the figures
│   └── plot_world.py        # floor plan of the worlds from the SDF
├── launch/                  # workshop, simulation, mapping
├── config/                  # robot, wheel controller, worlds
└── rviz/
```

## Validation

```bash
python3 src/omni_workshop/tools/test_kinematics.py
python3 src/omni_workshop/tools/validate_figures.py grupo14 RASC
```

## License

The base workshop material is by David Torres (Apache-2.0). The modifications
in this repository are by Team 14.
