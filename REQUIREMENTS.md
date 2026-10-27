# Workshop 03 requirements

## 1. Ubuntu 24.04 and ROS 2 Jazzy

- Ubuntu 24.04 LTS: [download and installation](https://ubuntu.com/download/desktop)
- ROS 2 Jazzy: [installation on Ubuntu (deb packages)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

## 2. Sourcing ROS 2

Every new terminal needs to load the ROS 2 environment:

```bash
source /opt/ros/jazzy/setup.bash
```

To load it automatically in every terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## 3. Clone this repository and download the Clearpath repositories

The repository is the `src/` folder of the workspace:

```bash
mkdir -p ~/workshop_03
git clone https://github.com/davatorr1009/workshop_03.git ~/workshop_03/src
```

The Clearpath repositories are **not** included in this repository: each student downloads them into their workspace with `vcs`, from `dependencies.repos` (it pins each repository to the `jazzy` branch commit the workshop was tested with).

```bash
sudo apt install python3-vcstool
cd ~/workshop_03/src
vcs import < dependencies.repos
```

This creates `src/clearpath_ws/src/` with 6 repositories (~1 GB): `clearpath_simulator`, `clearpath_common`, `clearpath_config`, `clearpath_msgs`, `clearpath_desktop` and `clearpath_nav2_demos` (navigation and mapping configuration for the Dingo-O, see section 4). To check that they are at the right version:

```bash
vcs log -l 1 clearpath_ws/src      # each one must show the commit from dependencies.repos
```

> If `vcs import` fails halfway (for example because of the network), you can run it again: it skips what is already downloaded.

## 4. Navigation and mapping

Nav2 (navigation) and slam_toolbox (mapping) packages:

```bash
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox
```

The configuration of these packages for the Dingo-O comes in `clearpath_nav2_demos`, which was already downloaded in section 3 and is built with the rest of the workspace.

## 5. Dependencies and build

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions python3-rosdep
cd ~/workshop_03
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys clearpath_generator_robot
colcon build --symlink-install
source install/setup.bash
```

- `--symlink-install` makes changes to Python files, launch files and worlds take effect without rebuilding.
- `--skip-keys clearpath_generator_robot` is needed because that package only exists on real Clearpath robots.
- If this is the first time `rosdep` is used on the computer, run `sudo rosdep init` first.
- If the computer freezes while building (low RAM): `MAKEFLAGS=-j2 colcon build --symlink-install --parallel-workers 2`.
- If `apt` replies *"Could not get lock /var/lib/dpkg/lock-frontend ... held by process (unattended-upgr)"*, Ubuntu is installing automatic updates: wait for them to finish (it can take several minutes) and run the command again. Do not delete the lock file or kill the process.

Once this is done, continue with the workshop in the [`README.md`](README.md).
