# Clearpath Dingo-O, con el controlador de Clearpath, en uno de los mundos del paquete
# worlds (lista y posicion inicial en config/mundos.yaml).
#   ros2 launch taller_omni simulacion.launch.py                    -> mundo por defecto
#   ros2 launch taller_omni simulacion.launch.py mundo:=oficina_1   -> otro mundo
#
# No usa clearpath_gz/simulation.launch.py porque solo acepta sus propios mundos.
# Ademas separa servidor y GUI de Gazebo: la GUI se abre cuando el robot ya
# existe. Si se abre antes (como en el launch de Clearpath), nunca recibe el
# componente SensorTopic de los lidars y el panel "Visualize Lidar" no encuentra
# el sensor, por lo que dibuja los rayos fijos en el origen del mundo.
import os
import shutil

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'do100_0000'
PKG = get_package_share_directory('taller_omni')
WORLDS = get_package_share_directory('worlds')
# Clearpath genera aqui (fuera de src/) el URDF, parametros y launch del robot
SETUP_PATH = os.path.expanduser('~/.ros/taller_omni/clearpath/')

with open(os.path.join(PKG, 'config', 'mundos.yaml')) as f:
    MUNDOS = yaml.safe_load(f)

ARGUMENTS = [
    DeclareLaunchArgument('mundo', default_value='oficina_3', choices=list(MUNDOS),
                          description='Mundo de config/mundos.yaml'),
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
    # Vacios: se usa el inicio del mundo en config/mundos.yaml
    DeclareLaunchArgument('x', default_value=''),
    DeclareLaunchArgument('y', default_value=''),
    DeclareLaunchArgument('z', default_value='0.3'),
    DeclareLaunchArgument('yaw', default_value=''),
]


def lanzar(context):
    clave = LaunchConfiguration('mundo').perform(context)
    mundo = MUNDOS[clave]
    sdf = os.path.join(WORLDS, 'worlds', f'{clave}.sdf')
    inicio = dict(zip(('x', 'y', 'yaw'), (str(v) for v in mundo['inicio'])))
    for eje in inicio:
        inicio[eje] = LaunchConfiguration(eje).perform(context) or inicio[eje]

    # El robot es el de config/dingo_o.yaml (igual para todos): se copia como
    # robot.yaml a la carpeta de trabajo de Clearpath.
    os.makedirs(SETUP_PATH, exist_ok=True)
    shutil.copy(os.path.join(PKG, 'config', 'dingo_o.yaml'), os.path.join(SETUP_PATH, 'robot.yaml'))

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                                 'launch', 'gz_sim.launch.py')

    # Igual que clearpath_gz/gz_sim.launch.py (mundos, mallas y todos los share/),
    # mas los modelos de los mundos del workshop.
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([os.path.join(WORLDS, 'models'),
                        os.path.join(pkg_clearpath_gz, 'worlds'),
                        os.path.join(pkg_clearpath_gz, 'meshes')] + packages_paths))

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[('gz_args', f'{sdf} -s -r -v 4')])

    # Espera a que el robot publique sus lidars y entonces abre la GUI.
    # Cerrar la ventana de Gazebo termina toda la simulacion.
    wait_robot = ExecuteProcess(
        cmd=['bash', '-c', f'until gz topic -l | grep -q "/{NAMESPACE}/sensors/lidar2d_1/scan"; '
                           'do sleep 1; done'])
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[('gz_args', '-g -v 4 --gui-config '
                           + os.path.join(PKG, 'config', 'gui.config')),
                          ('on_exit_shutdown', 'true')])
    gui_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=wait_robot, on_exit=[gz_gui]))

    # El servidor corre como 'sh -c ruby gz sim ...' y la senal de cierre no
    # siempre llega al proceso real: sin esto queda un servidor huerfano que
    # choca con el siguiente lanzamiento.
    kill_server = RegisterEventHandler(OnShutdown(on_shutdown=[ExecuteProcess(
        cmd=['pkill', '-f', f'gz sim {sdf} -s'])]))

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_gz, 'launch',
                                                   'robot_spawn.launch.py')),
        launch_arguments=[('use_sim_time', 'true'),
                          ('world', mundo['world']),
                          ('setup_path', SETUP_PATH),
                          ('rviz', 'false'),
                          ('x', inicio['x']),
                          ('y', inicio['y']),
                          ('z', LaunchConfiguration('z')),
                          ('yaw', inicio['yaw'])])

    return [resource_path, gz_server, robot_spawn, wait_robot, gui_after_robot, kill_server]


# RViz propio: robot + los dos lidars. El TF del robot va en /do100_0000/tf.
# Se evalua antes de los includes: robot_spawn recibe rviz:=false y, como los
# includes no aislan sus argumentos, despues 'rviz' ya valdria false aqui.
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
    ld.add_action(OpaqueFunction(function=lanzar))
    return ld
