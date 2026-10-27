"""Measures each run: desired trajectory, ground truth and odometry, in a single node.

Launched by workshop.launch.py. Publishes, in Gazebo's 'world' frame:
  /workshop/ground_truth      exact robot pose in Gazebo (Odometry)
  /workshop/odometry          odometry computed from the wheel velocities (Odometry)
  /workshop/path/desired      desired trajectory of the run (Path)
  /workshop/path/actual       actual path of the run (Path)
  /workshop/path/odometry     path according to the odometry (Path)
the world -> odom transform, to see everything together with the robot in RViz, and
/do100_0000/platform/odom (wheel odometry for Clearpath's EKF), which nobody else
publishes when the own controller is in use.

A run starts when the first reference arrives on /workshop/reference (in the 'start'
frame, where the robot started) and ends when they stop arriving. At the end it saves
a plot in ~/workshop_results/.
"""
import math
import os
import threading
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from gz.msgs10.empty_pb2 import Empty  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.msgs10.stringmsg_v_pb2 import StringMsg_V  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

ROBOT = '/do100_0000'
GZ_MODEL = 'do100_0000/robot'
RESULTS_DIR = os.path.expanduser('~/workshop_results')

# Dingo-O dimensions, for the forward kinematics
R, LX, LY = 0.05, 0.285, 0.1854
WHEELS = ('front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint')


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def integrate(pose, vx, vy, w, dt):
    """Advances the pose (x, y, theta) with the robot velocity (vx, vy, w) during dt."""
    x, y, th = pose
    return (x + (math.cos(th) * vx - math.sin(th) * vy) * dt,
            y + (math.sin(th) * vx + math.cos(th) * vy) * dt,
            th + w * dt)


def pose_stamped(stamp, x, y, th):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = 'world'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return msg


class Plotter(Node):

    def __init__(self):
        super().__init__('plotter', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.gz_pose = None                 # (x, y, theta) in Gazebo, written by gz-transport
        self.odom = None                    # (x, y, theta) integrated from the wheels
        self.t_wheels = None
        self.world_odom = None
        self.ekf = None                     # last pose of Clearpath's EKF (odom frame)
        self.platform_odom = None           # (x, y, theta) in the odom frame, for the EKF
        self.clearpath_active = True        # Clearpath's mecanum controller
        self.run = None                     # lists of the run in progress
        self.last_ref = None

        self.ground_truth_pub = self.create_publisher(Odometry, '/workshop/ground_truth', 10)
        self.odom_pub = self.create_publisher(Odometry, '/workshop/odometry', 10)
        # With the own controller, Clearpath's is switched off and nobody publishes the
        # wheel odometry its EKF uses: without it, the EKF only integrates the IMU and
        # odom -> base_link drifts even while the robot stands still.
        self.platform_pub = self.create_publisher(Odometry, f'{ROBOT}/platform/odom', 10)
        self.controllers_cli = self.create_client(ListControllers, f'{ROBOT}/controller_manager/list_controllers')
        self.controllers_timer = self.create_timer(0.5, self.check_controllers)
        self.path_pubs = {k: self.create_publisher(Path, f'/workshop/path/{k}', 10)
                          for k in ('desired', 'actual', 'odometry')}
        # The robot TF is namespaced: world -> odom is added on /do100_0000/tf_static
        self.tf_static_pub = self.create_publisher(TFMessage, f'{ROBOT}/tf_static', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(PoseStamped, '/workshop/reference', self.on_reference, 10)
        self.create_subscription(JointState, f'{ROBOT}/platform/joint_states', self.on_wheels, 50)
        self.create_subscription(Odometry, f'{ROBOT}/platform/odom/filtered', self.on_ekf, 10)
        self.create_timer(0.05, self.publish)
        self.create_timer(0.5, self.check_end)

        self.gz = GzNode()
        self.connect_gazebo()

    # --- ground truth (Gazebo) ---------------------------------------------
    def connect_gazebo(self):
        while rclpy.ok():
            ok, rep = self.gz.request('/gazebo/worlds', Empty(), Empty, StringMsg_V, 3000)
            if ok and rep.data:
                break
            self.get_logger().info('Waiting for Gazebo ...')
        topic = f'/world/{rep.data[0]}/dynamic_pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.on_gazebo):
            self.get_logger().error(f'Could not subscribe to {topic}')

    def on_gazebo(self, msg):
        for p in msg.pose:
            if p.name == GZ_MODEL:
                pose = (p.position.x, p.position.y, yaw(p.orientation))
                if math.isfinite(pose[2]):
                    with self.lock:
                        self.gz_pose = pose
                return

    # --- wheel odometry (forward kinematics) -------------------------------
    def on_wheels(self, msg):
        t = seconds(msg.header.stamp)
        try:
            w_fl, w_fr, w_rl, w_rr = (msg.velocity[msg.name.index(n)] for n in WHEELS)
        except (ValueError, IndexError):
            return
        if self.odom is None:
            with self.lock:
                self.odom = self.gz_pose      # starts at the ground truth pose
            return
        if self.t_wheels is None:
            self.t_wheels = t
            return
        dt, self.t_wheels = t - self.t_wheels, t
        if not 0 < dt < 0.5:
            return
        vx = R / 4 * (w_fl + w_fr + w_rl + w_rr)
        vy = R / 4 * (-w_fl + w_fr + w_rl - w_rr)
        w = R / (4 * (LX + LY)) * (-w_fl + w_fr - w_rl + w_rr)
        self.odom = integrate(self.odom, vx, vy, w, dt)
        self.publish_platform(msg.header.stamp, vx, vy, w, dt)

    def publish_platform(self, stamp, vx, vy, w, dt):
        if self.clearpath_active or self.ekf is None or self.gz_pose is None:
            return
        if self.platform_odom is None:
            # Takeover: continue where the EKF left off and recompute world -> odom with
            # the robot still, in case the odometry jumped while switching controllers
            self.platform_odom = self.ekf
            self.publish_world_odom(self.ekf, stamp)
            self.get_logger().info("Publishing wheel odometry for Clearpath's EKF")
        self.platform_odom = integrate(self.platform_odom, vx, vy, w, dt)
        x, y, th = self.platform_odom
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose = pose_stamped(stamp, x, y, th).pose
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
        msg.twist.twist.angular.z = w
        for i in range(6):
            msg.pose.covariance[7 * i] = 0.001
            msg.twist.covariance[7 * i] = 0.001
        self.platform_pub.publish(msg)

    def check_controllers(self):
        # Asks the controller_manager whether Clearpath's controller is still active
        if not self.controllers_cli.service_is_ready():
            return
        def response(future):
            states = {c.name: c.state for c in future.result().controller}
            if states.get('platform_velocity_controller', 'active') != 'active':
                self.clearpath_active = False
                self.controllers_timer.cancel()
        self.controllers_cli.call_async(ListControllers.Request()).add_done_callback(response)

    def on_ekf(self, msg):
        p = msg.pose.pose
        xo, yo, tho = p.position.x, p.position.y, yaw(p.orientation)
        self.ekf = (xo, yo, tho)
        # At startup, with the robot still
        if self.world_odom is None and self.gz_pose is not None:
            self.publish_world_odom(self.ekf, msg.header.stamp)

    def publish_world_odom(self, ekf, stamp):
        """world -> odom = gz_pose * odom_ekf^-1 (the robot must be still)."""
        xo, yo, tho = ekf
        with self.lock:
            xg, yg, thg = self.gz_pose
        th = thg - tho
        x = xg - (math.cos(th) * xo - math.sin(th) * yo)
        y = yg - (math.sin(th) * xo + math.cos(th) * yo)
        self.world_odom = (x, y, th)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id, tf.child_frame_id = 'world', 'odom'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.tf_static_pub.publish(TFMessage(transforms=[tf]))

    # --- runs ---------------------------------------------------------------
    def on_reference(self, msg):
        t = seconds(msg.header.stamp)
        with self.lock:
            actual = self.gz_pose
        if actual is None:
            return
        if self.run is None:
            # A run starts: the reference comes in the 'start' frame (where the robot
            # started) and the odometry starts at the ground truth pose
            self.start = actual
            self.odom = actual
            self.t_wheels = None
            self.run = {'desired': [], 'actual': [], 'odometry': []}
            self.paths = {k: Path() for k in self.run}
            self.get_logger().info('Run in progress ...')
        x0, y0, th0 = self.start
        xi, yi, thi = msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation)
        desired = (x0 + math.cos(th0) * xi - math.sin(th0) * yi,
                   y0 + math.sin(th0) * xi + math.cos(th0) * yi, th0 + thi)
        ref = (t, desired[0], desired[1])
        self.run['desired'].append(ref)
        self.run['actual'].append((t, actual[0], actual[1]))
        self.run['odometry'].append((t, self.odom[0], self.odom[1]))
        for k, (x, y, th) in (('desired', desired),
                              ('actual', actual), ('odometry', self.odom)):
            self.paths[k].poses.append(pose_stamped(msg.header.stamp, x, y, th))
        self.last_ref = time.monotonic()

    def check_end(self):
        if self.run is not None and time.monotonic() - self.last_ref > 2.0:
            self.save(self.run)
            self.run = None

    # --- publications ---------------------------------------------------------
    def publish(self):
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            actual = self.gz_pose
        for pub, pose in ((self.ground_truth_pub, actual), (self.odom_pub, self.odom)):
            if pose is None:
                continue
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id, msg.child_frame_id = 'world', 'base_link'
            msg.pose.pose = pose_stamped(stamp, *pose).pose
            pub.publish(msg)
        if self.run is not None:
            for k, path in self.paths.items():
                path.header.stamp, path.header.frame_id = stamp, 'world'
                self.path_pubs[k].publish(path)

    # --- plot -------------------------------------------------------------------
    def save(self, run):
        ref = run['desired']
        if len(ref) < 2:
            return
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
        a1.plot([r[1] for r in ref], [r[2] for r in ref], 'k--', lw=2, label='desired')
        summary = []
        for k, color in (('actual', 'C0'), ('odometry', 'C1')):
            points = run[k]
            errors = [math.hypot(r[1] - p[1], r[2] - p[2]) for r, p in zip(ref, points)]
            rms = math.sqrt(sum(e * e for e in errors) / len(errors))
            summary.append(f'{k}: RMS {rms:.3f} m')
            a1.plot([p[1] for p in points], [p[2] for p in points], color=color, lw=2, label=k)
            a2.plot([r[0] - ref[0][0] for r in ref], errors, color=color,
                    label=f'{k} (RMS {rms:.3f} m, max {max(errors):.3f} m)')
        a1.plot(ref[0][1], ref[0][2], 'ko', label='start')
        a1.set_aspect('equal'); a1.set_xlabel('x [m]'); a1.set_ylabel('y [m]')
        a1.set_title('Trajectory'); a1.legend(loc='best', fontsize=8); a1.grid(alpha=0.3)
        a2.set_xlabel('t [s]'); a2.set_ylabel('error with respect to the desired path [m]')
        a2.set_title('Position error'); a2.legend(fontsize=8); a2.grid(alpha=0.3)
        fig.tight_layout()
        os.makedirs(RESULTS_DIR, exist_ok=True)
        path = os.path.join(RESULTS_DIR, time.strftime('trajectory_%Y%m%d_%H%M%S.png'))
        fig.savefig(path, dpi=110)
        plt.close(fig)
        self.get_logger().info(f'{", ".join(summary)}. Plot saved to {path}')


def main():
    rclpy.init()
    node = Plotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.run is not None:
            node.save(node.run)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
