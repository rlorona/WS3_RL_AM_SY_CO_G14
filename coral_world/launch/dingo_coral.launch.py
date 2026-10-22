# Clearpath Dingo-O en el mundo CoRAL.
#
# No usa clearpath_gz/simulation.launch.py porque solo acepta sus propios mundos.
# Ademas separa servidor y GUI de Gazebo: la GUI se abre cuando el robot ya
# existe. Si se abre antes (como en el launch de Clearpath), nunca recibe el
# componente SensorTopic de los lidars y el panel "Visualize Lidar" no encuentra
# el sensor, por lo que dibuja los rayos fijos en el origen del mundo.
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'do100_0000'
# Clearpath genera aqui (fuera de src/) el URDF, parametros y launch del robot
SETUP_PATH = os.path.expanduser('~/.ros/coral_world/clearpath/')

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='1.5'),
    DeclareLaunchArgument('z', default_value='0.3'),
    DeclareLaunchArgument('yaw', default_value='0.0'),
]


def generate_launch_description():
    # El robot es el de config/dingo_o.yaml (igual para todos): se copia como
    # robot.yaml a la carpeta de trabajo de Clearpath.
    os.makedirs(SETUP_PATH, exist_ok=True)
    shutil.copy(os.path.join(get_package_share_directory('coral_world'), 'config', 'dingo_o.yaml'),
                os.path.join(SETUP_PATH, 'robot.yaml'))

    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_coral = get_package_share_directory('coral_world')
    gz_sim_launch = os.path.join(get_package_share_directory('ros_gz_sim'),
                                 'launch', 'gz_sim.launch.py')

    # Igual que clearpath_gz/gz_sim.launch.py: mundos, mallas y todos los share/
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([os.path.join(pkg_clearpath_gz, 'worlds'),
                        os.path.join(pkg_clearpath_gz, 'meshes')] + packages_paths))

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[('gz_args', 'coral.sdf -s -r -v 4')])

    # Espera a que el robot publique sus lidars y entonces abre la GUI.
    # Cerrar la ventana de Gazebo termina toda la simulacion.
    wait_robot = ExecuteProcess(
        cmd=['bash', '-c', f'until gz topic -l | grep -q "/{NAMESPACE}/sensors/lidar2d_1/scan"; '
                           'do sleep 1; done'])
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[('gz_args', '-g -v 4 --gui-config '
                           + os.path.join(pkg_coral, 'config', 'gui.config')),
                          ('on_exit_shutdown', 'true')])
    gui_after_robot = RegisterEventHandler(
        OnProcessExit(target_action=wait_robot, on_exit=[gz_gui]))

    # El servidor corre como 'sh -c ruby gz sim ...' y la senal de cierre no
    # siempre llega al proceso real: sin esto queda un servidor huerfano que
    # choca con el siguiente lanzamiento.
    kill_server = RegisterEventHandler(OnShutdown(on_shutdown=[ExecuteProcess(
        cmd=['pkill', '-f', 'gz sim coral.sdf -s'])]))

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_clearpath_gz, 'launch',
                                                   'robot_spawn.launch.py')),
        launch_arguments=[('use_sim_time', 'true'),
                          ('world', 'coral'),
                          ('setup_path', SETUP_PATH),
                          ('rviz', 'false'),
                          ('x', LaunchConfiguration('x')),
                          ('y', LaunchConfiguration('y')),
                          ('z', LaunchConfiguration('z')),
                          ('yaw', LaunchConfiguration('yaw'))])

    # RViz propio: robot + los dos lidars. El TF del robot va en /do100_0000/tf.
    # Se evalua antes de los includes: robot_spawn recibe rviz:=false y, como los
    # includes no aislan sus argumentos, despues 'rviz' ya valdria false aqui.
    def rviz_node(context):
        if LaunchConfiguration('rviz').perform(context) != 'true':
            return []
        return [Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(pkg_coral, 'rviz', 'dingo.rviz')],
            parameters=[{'use_sim_time': True}],
            remappings=[('/tf', f'/{NAMESPACE}/tf'), ('/tf_static', f'/{NAMESPACE}/tf_static')])]

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=rviz_node))
    ld.add_action(resource_path)
    ld.add_action(clock_bridge)
    ld.add_action(gz_server)
    ld.add_action(robot_spawn)
    ld.add_action(wait_robot)
    ld.add_action(gui_after_robot)
    ld.add_action(kill_server)
    return ld
