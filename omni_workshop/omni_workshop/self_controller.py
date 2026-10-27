"""Own controller: follows a figure from trajectory.py, computing the velocity of
each wheel yourself with the kinematic model of the omnidirectional robot.

    ros2 run omni_workshop self_controller --ros-args -p figure:=square

Requires the workshop simulation (ros2 launch omni_workshop workshop.launch.py),
which removes Clearpath's controller and leaves the wheels to you.

Your part: complete world_to_robot() and inverse_kinematics() (the README equations).
After saving there is no need to rebuild: just run the node again.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

from omni_workshop.trajectory import FIGURES

# Clearpath Dingo-O dimensions (do not change them)
R = 0.05      # wheel radius [m]
LX = 0.285    # distance from the center to the wheel axle (front/back) [m]
LY = 0.1854   # distance from the center to the wheel center (sideways) [m]

WHEELS_TOPIC = '/do100_0000/wheel_velocity_controller/commands'
RATE = 20.0   # Hz


def world_to_robot(theta, dx, dy, dtheta):
    """Velocity in the WORLD frame -> velocity in the ROBOT frame.

    theta: robot orientation. dx, dy, dtheta: desired velocity in the world.
    Returns (vx, vy, w) in the robot frame.
    """
    # >>> WRITE HERE <<<
    raise NotImplementedError('complete world_to_robot() in self_controller.py')


def inverse_kinematics(vx, vy, w):
    """Robot velocity (vx, vy, w) -> angular velocity of each wheel [rad/s].

    Returns (w_fl, w_fr, w_rl, w_rr): front left, front right, rear left and
    rear right. Use R, LX and LY.
    """
    # >>> WRITE HERE <<<
    raise NotImplementedError('complete inverse_kinematics() in self_controller.py')


class SelfController(Node):

    def __init__(self):
        super().__init__('self_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        name = self.declare_parameter('figure', 'square').value
        if name not in FIGURES:
            raise SystemExit(f'Figure "{name}" does not exist. Options: {", ".join(FIGURES)}')
        self.figure = FIGURES[name]()
        self.t0 = None
        self.ready_since = None
        self.wheels_pub = self.create_publisher(Float64MultiArray, WHEELS_TOPIC, 10)
        # The reference is in the 'start' frame (where the robot started); the plotter
        # places it in the world and compares it with the actual path and the odometry.
        self.reference_pub = self.create_publisher(PoseStamped, '/workshop/reference', 10)
        self.create_timer(1.0 / RATE, self.step)
        self.get_logger().info(f'Figure "{name}": {self.figure.duration:.1f} s. '
                               'Waiting for the simulation ...')

    def step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            # Wait for the simulation clock and the wheel controller; otherwise
            # the first commands are lost.
            if now == 0.0 or self.wheels_pub.get_subscription_count() == 0:
                return
            if self.ready_since is None:
                self.ready_since = now
            if now - self.ready_since < 1.0:
                return
            self.t0 = now
            self.get_logger().info('Running.')

        t = now - self.t0
        if t > self.figure.duration:
            self.send((0.0, 0.0, 0.0, 0.0))
            self.get_logger().info('Figure finished.')
            raise SystemExit

        x, y, theta, dx, dy, dtheta = self.figure(t)
        vx, vy, w = world_to_robot(theta, dx, dy, dtheta)
        self.send(inverse_kinematics(vx, vy, w))

        ref = PoseStamped()
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.header.frame_id = 'start'
        ref.pose.position.x, ref.pose.position.y = x, y
        ref.pose.orientation.z, ref.pose.orientation.w = math.sin(theta / 2), math.cos(theta / 2)
        self.reference_pub.publish(ref)

    def send(self, wheels):
        self.wheels_pub.publish(Float64MultiArray(data=[float(v) for v in wheels]))


def main():
    rclpy.init()
    node = None
    try:
        node = SelfController()
        rclpy.spin(node)
    except NotImplementedError as e:
        print(f'Your part is missing: {e}')
    except (KeyboardInterrupt, SystemExit) as e:
        if isinstance(e, SystemExit) and e.code:
            print(e.code)
    finally:
        if node is not None:
            node.send((0.0, 0.0, 0.0, 0.0))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
