# Simulation for part 1 of the workshop: Dingo-O with its OWN controller.
# Clearpath's mecanum controller is removed and the robot receives the velocity of
# each wheel directly (computed by self_controller with the kinematic model).
#   ros2 launch omni_workshop workshop.launch.py                 -> 6 x 6 m arena
#   ros2 launch omni_workshop workshop.launch.py rviz:=true      -> also opens RViz
# To use Clearpath's controller (cmd_vel, Gazebo panel, mapping):
#   ros2 launch omni_workshop simulation.launch.py
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
    pkg = get_package_share_directory('omni_workshop')
    wheel_params = os.path.join(pkg, 'config', 'wheel_controller.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'simulation.launch.py')),
        # The start pose of each world is in config/worlds.yaml
        launch_arguments=[('world', LaunchConfiguration('world')), ('rviz', 'false')])

    # Ground truth, odometry and paths of each run; saves the plot at the end
    plotter = Node(package='omni_workshop', executable='plotter', output='screen')

    # RViz with the robot and the three paths (desired, actual, odometry) in the world frame
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'workshop.rviz')],
        parameters=[{'use_sim_time': True}],
        remappings=[('/tf', '/do100_0000/tf'), ('/tf_static', '/do100_0000/tf_static')],
        condition=IfCondition(LaunchConfiguration('rviz')))

    # Once Clearpath's mecanum controller is active, it is replaced by one that
    # receives the velocity of each wheel.
    switch_controller = ExecuteProcess(
        cmd=['bash', '-c',
             f'until ros2 control list_controllers -c {CM} 2>/dev/null'
             ' | grep -q "platform_velocity_controller.*active"; do sleep 1; done; '
             f'ros2 run controller_manager spawner wheel_velocity_controller -c {CM}'
             f' -p {wheel_params} --inactive && '
             f'ros2 control switch_controllers -c {CM}'
             ' --deactivate platform_velocity_controller --activate wheel_velocity_controller'
             # Initial zero command: without one, Gazebo may receive NaN on the wheels
             ' && ros2 topic pub --once /do100_0000/wheel_velocity_controller/commands'
             ' std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0]}" > /dev/null'
             ' && echo "Wheel controller active: /do100_0000/wheel_velocity_controller/commands"'],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='arena'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        # rviz goes before the simulation: the include sets rviz:=false so it does not
        # open the RViz of simulation.launch.py, and includes do not isolate arguments
        rviz,
        simulation,
        plotter,
        switch_controller,
    ])
