# Simulador del taller: Dingo-O dentro de la jaula de drones del CoRAL.
#   ros2 launch taller_omni taller.launch.py nivel:=1   -> controlador mecanum (recibe cmd_vel)
#   ros2 launch taller_omni taller.launch.py nivel:=2   -> controlador por rueda (niveles 2 y 3)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

CM = '/do100_0000/controller_manager'


def generate_launch_description():
    nivel = LaunchConfiguration('nivel')
    params_ruedas = os.path.join(get_package_share_directory('taller_omni'),
                                 'config', 'ruedas_controller.yaml')

    simulador = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('coral_world'), 'launch', 'dingo_coral.launch.py')),
        launch_arguments=[('rviz', LaunchConfiguration('rviz')),
                          # Centro-abajo de la jaula: deja ~2 m libres hacia +y y +-2 m en x
                          ('x', '-4.875'), ('y', '-0.3'), ('yaw', '0.0')])

    pose_real = Node(package='taller_omni', executable='pose_real', output='screen')

    # Niveles 2 y 3: cuando el controlador mecanum de Clearpath ya este activo,
    # se reemplaza por uno que recibe la velocidad de cada rueda.
    cambiar_controlador = ExecuteProcess(
        cmd=['bash', '-c',
             f'until ros2 control list_controllers -c {CM} 2>/dev/null'
             ' | grep -q "platform_velocity_controller.*active"; do sleep 1; done; '
             f'ros2 run controller_manager spawner wheel_velocity_controller -c {CM}'
             f' -p {params_ruedas} --inactive && '
             f'ros2 control switch_controllers -c {CM}'
             ' --deactivate platform_velocity_controller --activate wheel_velocity_controller'
             ' && echo "Controlador por rueda activo: /do100_0000/wheel_velocity_controller/commands"'],
        output='screen',
        condition=IfCondition(PythonExpression([nivel, ' >= 2'])))

    return LaunchDescription([
        DeclareLaunchArgument('nivel', default_value='1', choices=['1', '2', '3']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        simulador,
        pose_real,
        cambiar_controlador,
    ])
