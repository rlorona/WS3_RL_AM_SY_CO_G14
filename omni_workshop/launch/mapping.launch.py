# Part 2 of the workshop: mapping with Clearpath's controller.
# Requires the simulation to be running:  ros2 launch omni_workshop simulation.launch.py world:=<world>
#   ros2 launch omni_workshop mapping.launch.py              -> SLAM + RViz with the map
#   ros2 launch omni_workshop mapping.launch.py rviz:=false  -> SLAM only
#
# The robot TF is on /do100_0000/tf (and SLAM publishes map -> odom there), so
# RViz is launched with those remappings.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAMESPACE = 'do100_0000'
# Folder where simulation.launch.py leaves the generated robot configuration
SETUP_PATH = os.path.expanduser('~/.ros/omni_workshop/clearpath/')


def generate_launch_description():
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('omni_workshop'), 'rviz', 'mapping.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[('/tf', f'/{NAMESPACE}/tf'), ('/tf_static', f'/{NAMESPACE}/tf_static')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('clearpath_nav2_demos'), 'launch', 'slam.launch.py')),
        launch_arguments=[('setup_path', SETUP_PATH), ('use_sim_time', 'true')])

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        # rviz goes before the include, which could redefine the 'rviz' argument
        rviz,
        slam,
    ])
