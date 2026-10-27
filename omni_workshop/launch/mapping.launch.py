# Parte 2 del taller: mapeo con el controlador de Clearpath.
# Requiere la simulacion abierta con:  ros2 launch taller_omni simulacion.launch.py mundo:=<mundo>
#   ros2 launch taller_omni mapeo.launch.py              -> SLAM + RViz con el mapa
#   ros2 launch taller_omni mapeo.launch.py rviz:=false  -> solo SLAM
#
# El TF del robot va en /do100_0000/tf (y el SLAM publica ahi map -> odom), por eso
# RViz se lanza con esos remapeos.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'do100_0000'
# Carpeta donde simulacion.launch.py deja la configuracion generada del robot
SETUP_PATH = os.path.expanduser('~/.ros/taller_omni/clearpath/')


def generate_launch_description():
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('taller_omni'), 'rviz', 'mapeo.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[('/tf', f'/{NAMESPACE}/tf'), ('/tf_static', f'/{NAMESPACE}/tf_static')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('clearpath_nav2_demos'), 'launch', 'slam.launch.py')),
        launch_arguments=[('setup_path', SETUP_PATH), ('use_sim_time', 'true')])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        # rviz va antes del include, que podria redefinir el argumento 'rviz'
        rviz,
        slam,
    ])
