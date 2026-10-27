# Simulador de la parte 1 del taller: Dingo-O con el controlador PROPIO.
# Se quita el controlador mecanum de Clearpath y el robot recibe directamente la
# velocidad de cada rueda (la calcula self_controller con el modelo cinematico).
#   ros2 launch taller_omni taller.launch.py                 -> arena de 6 x 6 m
#   ros2 launch taller_omni taller.launch.py rviz:=true      -> ademas abre RViz
# Para usar el controlador de Clearpath (cmd_vel, panel de Gazebo, mapeo):
#   ros2 launch taller_omni simulacion.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CM = '/do100_0000/controller_manager'


def generate_launch_description():
    params_ruedas = os.path.join(get_package_share_directory('taller_omni'),
                                 'config', 'ruedas_controller.yaml')

    simulador = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('taller_omni'), 'launch', 'simulacion.launch.py')),
        # La posicion inicial de cada mundo esta en config/mundos.yaml
        launch_arguments=[('mundo', LaunchConfiguration('mundo')), ('rviz', 'false')])

    # Pose real, odometria y caminos de cada intento; guarda el grafico al terminar
    graficar = Node(package='taller_omni', executable='graficar', output='screen')

    # RViz con el robot y los tres caminos (deseado, real, odometria) en el marco world
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('taller_omni'), 'rviz', 'taller.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[('/tf', '/do100_0000/tf'), ('/tf_static', '/do100_0000/tf_static')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    # Cuando el controlador mecanum de Clearpath ya este activo, se reemplaza por
    # uno que recibe la velocidad de cada rueda.
    cambiar_controlador = ExecuteProcess(
        cmd=['bash', '-c',
             f'until ros2 control list_controllers -c {CM} 2>/dev/null'
             ' | grep -q "platform_velocity_controller.*active"; do sleep 1; done; '
             f'ros2 run controller_manager spawner wheel_velocity_controller -c {CM}'
             f' -p {params_ruedas} --inactive && '
             f'ros2 control switch_controllers -c {CM}'
             ' --deactivate platform_velocity_controller --activate wheel_velocity_controller'
             # Comando inicial en cero: sin ninguno, Gazebo puede recibir NaN en las ruedas
             ' && ros2 topic pub --once /do100_0000/wheel_velocity_controller/commands'
             ' std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}" > /dev/null'
             ' && echo "Controlador por rueda activo: /do100_0000/wheel_velocity_controller/commands"'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('mundo', default_value='arena'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        # rviz va antes del simulador: el include fija rviz:=false para no abrir el
        # RViz de simulacion.launch.py, y los includes no aislan sus argumentos
        rviz,
        simulador,
        graficar,
        cambiar_controlador,
    ])
