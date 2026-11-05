#!/usr/bin/env python3
"""Automatic exploration for the mapping part (bonus, not required).

It drives the robot with Clearpath's controller (it only publishes cmd_vel) and
picks goals on the frontier of the SLAM map: free cells next to unknown cells.
The robot is holonomic, so it moves sideways instead of turning, and the 360
degree lidar keeps seeing around it.

    # terminal 1
    ros2 launch omni_workshop simulation.launch.py world:=office_0
    # terminal 2
    ros2 launch omni_workshop mapping.launch.py rviz:=true
    # terminal 3
    ros2 run omni_workshop explore --ros-args -p time_limit:=600.0

When it prints "EXPLORATION FINISHED", save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \
        --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

try:
    from scipy.ndimage import distance_transform_edt, uniform_filter
except ImportError:  # scipy is not a hard dependency
    distance_transform_edt = None
    uniform_filter = None


class Explorer(Node):

    def __init__(self):
        super().__init__('explore', parameter_overrides=[Parameter('use_sim_time', value=True)])

        self.declare_parameter('namespace', '/do100_0000')
        # Clearpath's twist_mux runs with use_stamped:=True, so it only accepts
        # geometry_msgs/TwistStamped on cmd_vel (a plain Twist is ignored).
        self.declare_parameter('stamped', True)
        self.declare_parameter('vmax', 0.40)          # m/s
        self.declare_parameter('time_limit', 600.0)   # s of wall clock
        self.declare_parameter('goal_tolerance', 0.35)
        self.declare_parameter('clearance', 0.45)     # m around walls
        self.declare_parameter('min_goal_distance', 0.8)
        self.declare_parameter('max_goal_distance', 12.0)
        # Exploration radius around the starting pose: without it the robot
        # leaves the building through the door and explores the empty plane.
        self.declare_parameter('max_radius', 8.0)
        # Rectangular bound of the area to map, in the map frame (the map frame
        # starts at the robot pose). Frontiers outside it are ignored, which
        # keeps the robot from leaving the building through an open door.
        self.declare_parameter('x_min', -1e9)
        self.declare_parameter('x_max', 1e9)
        self.declare_parameter('y_min', -1e9)
        self.declare_parameter('y_max', 1e9)

        self.ns = self.get_parameter('namespace').value
        self.stamped = bool(self.get_parameter('stamped').value)
        self.vmax = float(self.get_parameter('vmax').value)
        self.time_limit = float(self.get_parameter('time_limit').value)
        self.tol = float(self.get_parameter('goal_tolerance').value)
        self.clearance = float(self.get_parameter('clearance').value)
        self.dmin = float(self.get_parameter('min_goal_distance').value)
        self.dmax = float(self.get_parameter('max_goal_distance').value)
        self.max_radius = float(self.get_parameter('max_radius').value)
        self.x_min = float(self.get_parameter('x_min').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_min = float(self.get_parameter('y_min').value)
        self.y_max = float(self.get_parameter('y_max').value)
        self.origin = None

        self.map = None
        self.grid = None
        self.inflated = None
        self.inflated_stamp = 0.0
        self.scan = None
        self.goal = None
        self.goal_since = 0.0
        self.last_pose = None
        self.last_move = time.monotonic()
        self.empty_since = None
        self.finished = False

        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, f'{self.ns}/cmd_vel', 10)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, f'{self.ns}/map', self.on_map, map_qos)
        self.create_subscription(LaserScan, f'{self.ns}/scan', self.on_scan, 10)

        # The robot TF is namespaced, so it is not on /tf: feed a buffer by hand
        from tf2_ros import Buffer
        self.buffer = Buffer(node=self)
        self.create_subscription(TFMessage, f'{self.ns}/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, f'{self.ns}/tf_static', self.on_tf, 100)

        self.start = time.monotonic()
        self.create_timer(0.1, self.step)
        self.get_logger().info('Waiting for the map, the lidar and the TF tree ...')

    # --- inputs -------------------------------------------------------------
    def on_tf(self, msg):
        for tf in msg.transforms:
            self.buffer.set_transform(tf, 'explore')

    def on_map(self, msg):
        self.map = msg
        self.grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def on_scan(self, msg):
        self.scan = msg

    def pose(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    # --- map helpers --------------------------------------------------------
    def cell_to_world(self, ix, iy):
        info = self.map.info
        return (info.origin.position.x + (ix + 0.5) * info.resolution,
                info.origin.position.y + (iy + 0.5) * info.resolution)

    def world_to_cell(self, x, y):
        info = self.map.info
        return (int((x - info.origin.position.x) / info.resolution),
                int((y - info.origin.position.y) / info.resolution))

    def build_inflated(self):
        """Occupancy grid dilated by the clearance (rebuilt at most every 2 s)."""
        now = time.monotonic()
        if self.inflated is not None and now - self.inflated_stamp < 2.0:
            return
        occupied = self.grid >= 50
        if distance_transform_edt is not None:
            dist = distance_transform_edt(~occupied) * self.map.info.resolution
            self.inflated = dist < self.clearance
        else:
            self.inflated = occupied.copy()
            k = max(1, int(self.clearance / self.map.info.resolution))
            for _ in range(k):
                self.inflated |= np.roll(self.inflated, 1, 0) | np.roll(self.inflated, -1, 0)
                self.inflated |= np.roll(self.inflated, 1, 1) | np.roll(self.inflated, -1, 1)
        self.inflated_stamp = now

    def line_of_sight(self, x0, y0, x1, y1):
        self.build_inflated()
        steps = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.10))
        for i in range(steps + 1):
            x = x0 + (x1 - x0) * i / steps
            y = y0 + (y1 - y0) * i / steps
            ix, iy = self.world_to_cell(x, y)
            if not (0 <= ix < self.grid.shape[1] and 0 <= iy < self.grid.shape[0]):
                return False
            if self.inflated[iy, ix]:
                return False
        return True

    # --- frontier -----------------------------------------------------------
    def choose_goal(self, pose):
        x, y, _ = pose
        if self.origin is None:
            self.origin = (x, y)
        res = self.map.info.resolution
        step = max(1, int(0.25 / res))
        g = self.grid[::step, ::step]
        free = g == 0
        unknown = g < 0
        frontier = np.zeros_like(free)
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            frontier |= free & np.roll(unknown, (dy, dx), axis=(0, 1))

        ys, xs = np.nonzero(frontier)
        if len(xs) == 0:
            return None

        # Information gain: unknown cells in a 1 m window (cheap box sum)
        k = max(1, int(0.5 / (res * step)))
        u = unknown.astype(np.float32)
        if uniform_filter is not None:
            gain = uniform_filter(u, size=2 * k + 1, mode='constant') * (2 * k + 1) ** 2
        else:
            gain = u

        best, best_score = None, -1e9
        for iy, ix in zip(ys, xs):
            wx, wy = self.cell_to_world(ix * step, iy * step)
            d = math.hypot(wx - x, wy - y)
            if not (self.dmin <= d <= self.dmax):
                continue
            if math.hypot(wx - self.origin[0], wy - self.origin[1]) > self.max_radius:
                continue
            if not (self.x_min <= wx <= self.x_max and self.y_min <= wy <= self.y_max):
                continue
            if not self.line_of_sight(x, y, wx, wy):
                continue
            score = float(gain[iy, ix]) - 0.04 * d
            if score > best_score:
                best, best_score = (wx, wy), score
        return best

    # --- control ------------------------------------------------------------
    def publish(self, vx, vy, wz=0.0):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, wz
        else:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def obstacle_ahead(self, ex, ey):
        """True if the lidar sees something close in the direction of motion."""
        if self.scan is None or math.hypot(ex, ey) < 1e-6:
            return False
        angle = math.atan2(ey, ex)
        a0 = self.scan.angle_min
        inc = self.scan.angle_increment
        ranges = self.scan.ranges
        for i, r in enumerate(ranges):
            a = a0 + i * inc
            da = math.atan2(math.sin(a - angle), math.cos(a - angle))
            if abs(da) < math.radians(18) and math.isfinite(r) and r < 0.55:
                return True
        return False

    def step(self):
        if self.finished:
            return
        pose = self.pose()
        if pose is None or self.map is None or self.grid is None:
            return

        now = time.monotonic()
        if now - self.start > self.time_limit:
            self.finish('time limit reached')
            return

        if self.goal is not None:
            gx, gy = self.goal
            if math.hypot(gx - pose[0], gy - pose[1]) < self.tol:
                self.goal = None
            elif now - self.goal_since > 45.0:
                self.get_logger().warn('Goal not reached in 45 s, replanning')
                self.goal = None

        if self.goal is None:
            if now - getattr(self, 'last_plan', 0.0) < 1.0:
                return
            self.last_plan = now
            self.goal = self.choose_goal(pose)
            self.goal_since = now
            if self.goal is None:
                # At startup the map is still tiny: give it time before saying
                # that there is nothing left to explore.
                if self.empty_since is None:
                    self.empty_since = now
                elif now - self.empty_since > 20.0:
                    self.finish('no frontiers left')
                return
            self.empty_since = None
            self.get_logger().info(
                f'New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f}), '
                f'{math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1]):.2f} m away')

        x, y, yaw = pose
        dx, dy = self.goal[0] - x, self.goal[1] - y
        ex = math.cos(yaw) * dx + math.sin(yaw) * dy
        ey = -math.sin(yaw) * dx + math.cos(yaw) * dy
        dist = math.hypot(ex, ey)
        if dist < 1e-6:
            return

        # Stuck detector: commanded to move but the pose does not change
        if self.last_pose is not None:
            if math.hypot(x - self.last_pose[0], y - self.last_pose[1]) > 0.05:
                self.last_move = now
        self.last_pose = (x, y)
        if now - self.last_move > 8.0:
            self.get_logger().warn('Stuck, replanning')
            self.goal = None
            self.last_move = now
            self.stop()
            return

        if self.obstacle_ahead(ex, ey):
            self.goal = None
            self.stop()
            return

        v = min(self.vmax, max(0.12, 0.9 * dist))
        self.publish(ex / dist * v, ey / dist * v)

    def finish(self, reason):
        self.finished = True
        self.stop()
        self.get_logger().info(f'EXPLORATION FINISHED: {reason}. '
                               'Save the map with map_saver_cli.')


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
