# Clearpath Dingo-O, with the Clearpath controller, in one of the worlds of the
# worlds package (list and start pose in config/worlds.yaml).
#   ros2 launch omni_workshop simulation.launch.py                    -> default world
#   ros2 launch omni_workshop simulation.launch.py world:=office_1    -> another world
#
# It does not use clearpath_gz/simulation.launch.py because that one only accepts
# Clearpath's own worlds. It also splits the Gazebo server and GUI: the GUI opens once
# the robot exists. If it opens earlier (as in Clearpath's launch), it never receives
# the lidars' SensorTopic component and the "Visualize Lidar" panel cannot find the
# sensor, so it draws the rays fixed at the world origin.
import os
import re
import shutil
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'do100_0000'
PKG = get_package_share_directory('omni_workshop')
WORLDS = get_package_share_directory('worlds')
# Clearpath generates the robot URDF, parameters and launch files here (outside src/)
SETUP_PATH = os.path.expanduser('~/.ros/omni_workshop/clearpath/')

with open(os.path.join(PKG, 'config', 'worlds.yaml')) as f:
    WORLD_LIST = yaml.safe_load(f)

ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='office_3', choices=list(WORLD_LIST),
                          description='World from config/worlds.yaml'),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
    # gui:=false runs only the Gazebo server (useful on slow machines or to
    # generate data without a window)
    DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
    # Robot description from config/: dingo_o.yaml has the two lidars of the
    # workshop; dingo_o_fast.yaml has a single lidar for low-resource machines.
    DeclareLaunchArgument('robot_config', default_value='dingo_o.yaml'),
    # false: reuse the description already generated in ~/.ros/omni_workshop
    # (skips the generator, useful to restart quickly or after patching it)
    DeclareLaunchArgument('generate', default_value='true', choices=['true', 'false']),
    # Lidar settings of the generated description. The stock values (40 Hz and
    # 0.5 deg, 720 rays) are what Clearpath uses; on a 4-core CPU without a GPU
    # they cost ~85 % of the simulation time. lidar_rate:=20 lidar_ang_res:=1.0
    # is 4x cheaper and still fine for SLAM (see docs/GUIA_EJECUCION.md).
    DeclareLaunchArgument('lidar_rate', default_value='40'),
    DeclareLaunchArgument('lidar_ang_res', default_value='0.5'),
    # Empty: use the world's start pose from config/worlds.yaml
    DeclareLaunchArgument('x', default_value=''),
    DeclareLaunchArgument('y', default_value=''),
    DeclareLaunchArgument('z', default_value='0.3'),
    DeclareLaunchArgument('yaw', default_value=''),
]


def patch_lidar_xacro(path, rate, ang_res):
    """Rewrites the lidar rate and angular resolution of the generated xacro.

    Clearpath's generator hardcodes 40 Hz and 0.5 deg (720 rays per scan) in
    clearpath_generator_common, so the only way to change them without patching
    that package is to edit the generated file before it is used.
    """
    with open(path) as f:
        text = f.read()
    text = re.sub(r'(<xacro:hokuyo_ust[^>]*?)ang_res="[^"]*"',
                  r'\1ang_res="%s"' % ang_res, text)
    text = re.sub(r'(<xacro:hokuyo_ust[^>]*?)update_rate="[^"]*"',
                  r'\1update_rate="%s"' % rate, text)
    with open(path, 'w') as f:
        f.write(text)


def launch_world(context):
    key = LaunchConfiguration('world').perform(context)
    world = WORLD_LIST[key]
    sdf = os.path.join(WORLDS, 'worlds', f'{key}.sdf')
    start = dict(zip(('x', 'y', 'yaw'), (str(v) for v in world['start'])))
    for axis in start:
        start[axis] = LaunchConfiguration(axis).perform(context) or start[axis]

    # The robot is the one in config/<robot_config> (the same for everyone by
    # default): it is copied as robot.yaml into Clearpath's working folder.
    robot_config = LaunchConfiguration('robot_config').perform(context)
    os.makedirs(SETUP_PATH, exist_ok=True)
    shutil.copy(os.path.join(PKG, 'config', robot_config),
                os.path.join(SETUP_PATH, 'robot.yaml'))

    # Custom lidar settings: generate the description here (the generator would
    # hardcode 40 Hz / 0.5 deg), patch it and tell robot_spawn not to regenerate.
    generate = LaunchConfiguration('generate').perform(context)
    rate = float(LaunchConfiguration('lidar_rate').perform(context))
    ang_res = float(LaunchConfiguration('lidar_ang_res').perform(context))
    spawn_generate = generate
    if generate == 'true' and (rate != 40.0 or ang_res != 0.5):
        generators = (('clearpath_generator_common', 'generate_description'),
                      ('clearpath_generator_common', 'generate_semantic_description'),
                      ('clearpath_generator_gz', 'generate_launch'),
                      ('clearpath_generator_gz', 'generate_param'))
        for pkg, exe in generators:
            subprocess.run(['ros2', 'run', pkg, exe, '-s', SETUP_PATH],
                           check=True, stdout=subprocess.DEVNULL)
        patch_lidar_xacro(os.path.join(SETUP_PATH, 'robot.urdf.xacro'), rate, ang_res)
        spawn_generate = 'false'
        print(f'[omni_workshop] lidar: {rate:g} Hz, {ang_res:g} deg, robot: {robot_config}')

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                                 'launch', 'gz_sim.launch.py')

    # Same as clearpath_gz/gz_sim.launch.py (worlds, meshes and every share/),
    # plus the models of the workshop worlds.
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([os.path.join(WORLDS, 'models'),
                        os.path.join(pkg_clearpath_gz, 'worlds'),
                        os.path.join(pkg_clearpath_gz, 'meshes')] + packages_paths))

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[('gz_args', f'{sdf} -s -r -v 4')])

    # Waits for the robot to publish its lidars and then opens the GUI.
    # Closing the Gazebo window ends the whole simulation.
    gui = IfCondition(LaunchConfiguration('gui'))
    wait_robot = ExecuteProcess(
        cmd=['bash', '-c', f'until gz topic -l | grep -q "/{NAMESPACE}/sensors/lidar2d_"; '
                           'do sleep 1; done'], condition=gui)
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[('gz_args', '-g -v 4 --gui-config '
                           + os.path.join(PKG, 'config', 'gui.config')),
                          ('on_exit_shutdown', 'true')])
    gui_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=wait_robot, on_exit=[gz_gui]), condition=gui)

    # The server runs as 'sh -c ruby gz sim ...' and the shutdown signal does not
    # always reach the real process: without this an orphan server remains and
    # collides with the next launch.
    kill_server = RegisterEventHandler(OnShutdown(on_shutdown=[ExecuteProcess(
        cmd=['pkill', '-f', f'gz sim {sdf} -s'])]))

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_gz, 'launch',
                                                   'robot_spawn.launch.py')),
        launch_arguments=[('use_sim_time', 'true'),
                          ('world', world['gz_world']),
                          ('setup_path', SETUP_PATH),
                          ('generate', spawn_generate),
                          ('rviz', 'false'),
                          ('x', start['x']),
                          ('y', start['y']),
                          ('z', LaunchConfiguration('z')),
                          ('yaw', start['yaw'])])

    return [resource_path, gz_server, robot_spawn, wait_robot, gui_after_robot, kill_server]


# Own RViz: robot + both lidars. The robot TF is on /do100_0000/tf.
# Evaluated before the includes: robot_spawn receives rviz:=false and, since
# includes do not isolate their arguments, 'rviz' would be false here afterwards.
def rviz_node(context):
    if LaunchConfiguration('rviz').perform(context) != 'true':
        return []
    return [Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(PKG, 'rviz', 'dingo.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[('/tf', f'/{NAMESPACE}/tf'), ('/tf_static', f'/{NAMESPACE}/tf_static')])]


def generate_launch_description():
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=rviz_node))
    ld.add_action(clock_bridge)
    ld.add_action(OpaqueFunction(function=launch_world))
    return ld
