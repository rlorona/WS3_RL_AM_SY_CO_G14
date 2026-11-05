#!/usr/bin/env python3
"""Coverage driver for the mapping part of the workshop.

The frontier explorer (`explore`) is nice, but in these worlds the building has
open doors to the outside: a frontier explorer follows them and ends up mapping
the empty plane around the building. This node instead plans a coverage route
over the *known* floor plan (the collision boxes of the world models), drives it
with feedback on the robot pose and lets SLAM build the map.

    ros2 launch omni_workshop simulation.launch.py world:=office_0
    ros2 launch omni_workshop mapping.launch.py rviz:=false
    ros2 run omni_workshop coverage --ros-args -p world:=office_0

When it prints COVERAGE FINISHED, save the map:

    ros2 run nav2_map_server map_saver_cli -f ~/workshop_results/maps/office_0 \\
        --ros-args -r map:=/do100_0000/map -p use_sim_time:=true
"""
import heapq
import math
from collections import deque
import os
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

RES = 0.10          # grid resolution [m]
VIEW_RADIUS = 1.4   # cells this close to the robot count as observed [m]


def parse_pose(text):
    v = [float(x) for x in (text or '0 0 0 0 0 0').split()]
    return v + [0.0] * (6 - len(v))


def load_boxes(world_sdf, models_dir):
    """Every box collision of the world, in world coordinates: (cx, cy, sx, sy, yaw)."""
    out = []
    root = ET.parse(world_sdf).getroot()
    for inc in root.iter('include'):
        uri = inc.findtext('uri', '')
        if not uri.startswith('model://'):
            continue
        name = uri[len('model://'):].split('/')[0]
        model_sdf = os.path.join(models_dir, name, 'model.sdf')
        if not os.path.exists(model_sdf):
            continue
        ip = parse_pose(inc.findtext('pose'))
        for c in ET.parse(model_sdf).getroot().iter('collision'):
            size = c.findtext('geometry/box/size')
            if not size:
                continue
            sx, sy, sz = (float(v) for v in size.split())
            if sz < 0.3:            # floors
                continue
            p = parse_pose(c.findtext('pose'))
            ca, sa = math.cos(ip[5]), math.sin(ip[5])
            cx = ip[0] + ca * p[0] - sa * p[1]
            cy = ip[1] + sa * p[0] + ca * p[1]
            out.append((cx, cy, sx, sy, ip[5] + p[5]))
    return out


class Coverage(Node):

    def __init__(self):
        super().__init__('coverage', parameter_overrides=[Parameter('use_sim_time', value=True)])

        self.declare_parameter('world', 'office_0')
        self.declare_parameter('namespace', '/do100_0000')
        self.declare_parameter('vmax', 0.35)
        self.declare_parameter('robot_radius', 0.40)
        self.declare_parameter('path_limit', 500.0)   # stop after this distance [m]
        self.declare_parameter('time_limit', 1800.0)  # wall clock [s]

        self.world = self.get_parameter('world').value
        self.ns = self.get_parameter('namespace').value
        self.vmax = float(self.get_parameter('vmax').value)
        self.radius = float(self.get_parameter('robot_radius').value)
        self.path_limit = float(self.get_parameter('path_limit').value)
        self.time_limit = float(self.get_parameter('time_limit').value)

        worlds = get_package_share_directory('worlds')
        cfg = os.path.join(get_package_share_directory('omni_workshop'),
                           'config', 'worlds.yaml')
        with open(cfg) as f:
            self.start = yaml.safe_load(f)[self.world]['start']

        boxes = load_boxes(os.path.join(worlds, 'worlds', f'{self.world}.sdf'),
                           os.path.join(worlds, 'models'))
        self.build_grid(boxes)

        self.pub = self.create_publisher(TwistStamped, f'{self.ns}/cmd_vel', 10)
        from tf2_ros import Buffer
        self.buffer = Buffer(node=self)
        self.create_subscription(TFMessage, f'{self.ns}/tf', self.on_tf, 100)
        self.create_subscription(TFMessage, f'{self.ns}/tf_static', self.on_tf, 100)

        self.path = []
        self.visited = np.zeros_like(self.grid, dtype=bool)
        self.distance = 0.0
        self.last_pose = None
        self.last_progress = 0.0
        self.observed_at = 0.0
        self.started = False
        self.finished = False
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.1, self.step)
        self.get_logger().info(
            f'{self.world}: {self.free.sum() * RES * RES:.1f} m2 of free floor, '
            f'{self.nx}x{self.ny} cells. Waiting for the robot pose ...')

    # --- grid ---------------------------------------------------------------
    def build_grid(self, boxes):
        xs = [b[0] for b in boxes]
        ys = [b[1] for b in boxes]
        self.x0 = min(xs) - 0.3
        self.y0 = min(ys) - 0.3
        self.nx = int((max(xs) + 0.3 - self.x0) / RES)
        self.ny = int((max(ys) + 0.3 - self.y0) / RES)
        self.grid = np.zeros((self.ny, self.nx), dtype=bool)
        gx = self.x0 + (np.arange(self.nx) + 0.5) * RES
        gy = self.y0 + (np.arange(self.ny) + 0.5) * RES
        for cx, cy, sx, sy, yaw in boxes:
            ca, sa = math.cos(yaw), math.sin(yaw)
            dx, dy = gx[None, :] - cx, gy[:, None] - cy
            lx = ca * dx + sa * dy
            ly = -sa * dx + ca * dy
            self.grid |= (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
        # inflate by the robot radius
        try:
            from scipy.ndimage import distance_transform_edt
            dist = distance_transform_edt(~self.grid) * RES
            self.blocked = dist < self.radius
        except ImportError:
            self.blocked = self.grid.copy()
            k = int(math.ceil(self.radius / RES))
            for _ in range(k):
                b = self.blocked
                self.blocked = (b | np.roll(b, 1, 0) | np.roll(b, -1, 0)
                                | np.roll(b, 1, 1) | np.roll(b, -1, 1))
        self.free = ~self.blocked

    def to_cell(self, x, y):
        return (int((x - self.x0) / RES), int((y - self.y0) / RES))

    def to_world(self, ix, iy):
        return (self.x0 + (ix + 0.5) * RES, self.y0 + (iy + 0.5) * RES)

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    # --- pose ---------------------------------------------------------------
    def on_tf(self, msg):
        for tf in msg.transforms:
            self.buffer.set_transform(tf, 'coverage')

    def pose(self):
        try:
            tf = self.buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # odom frame = start pose of the robot
        return t.x + self.start[0], t.y + self.start[1], yaw

    # --- planning -----------------------------------------------------------
    def neighbours(self, ix, iy):
        for dx, dy, cost in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                             (1, 1, 1.4142), (1, -1, 1.4142),
                             (-1, 1, 1.4142), (-1, -1, 1.4142)):
            nx, ny = ix + dx, iy + dy
            if self.inside(nx, ny) and self.free[ny, nx]:
                yield nx, ny, cost

    def astar(self, start, goal):
        open_set = [(0.0, start)]
        came, g = {}, {start: 0.0}
        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1]
            for nx, ny, c in self.neighbours(*cur):
                nxt = (nx, ny)
                ng = g[cur] + c
                if ng < g.get(nxt, 1e18):
                    g[nxt] = ng
                    came[nxt] = cur
                    h = math.hypot(nx - goal[0], ny - goal[1])
                    heapq.heappush(open_set, (ng + h, nxt))
        return []

    def bfs_distances(self, start):
        dist = {start: 0.0}
        queue = deque([start])
        while queue:
            cur = queue.popleft()
            for nx, ny, c in self.neighbours(*cur):
                nxt = (nx, ny)
                if nxt not in dist:
                    dist[nxt] = dist[cur] + c
                    queue.append(nxt)
        return dist

    def line_of_sight(self, a, b):
        steps = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) * 1.5))
        for i in range(steps + 1):
            x = a[0] + (b[0] - a[0]) * i / steps
            y = a[1] + (b[1] - a[1]) * i / steps
            ix, iy = int(x), int(y)
            if not self.inside(ix, iy) or self.blocked[iy, ix]:
                return False
        return True

    def mark_observed(self, cell):
        k = int(VIEW_RADIUS / RES)
        cx, cy = cell
        for iy in range(max(0, cy - k), min(self.ny, cy + k + 1)):
            for ix in range(max(0, cx - k), min(self.nx, cx + k + 1)):
                if self.free[iy, ix] and not self.visited[iy, ix]:
                    if math.hypot(ix - cx, iy - cy) * RES <= VIEW_RADIUS:
                        if self.line_of_sight((cx, cy), (ix, iy)):
                            self.visited[iy, ix] = True

    def plan_next(self, pose):
        cell = self.to_cell(pose[0], pose[1])
        if not self.inside(*cell) or not self.free[cell[1], cell[0]]:
            cell = self.nearest_free(cell)
            if cell is None:
                return None
        self.mark_observed(cell)
        dist = self.bfs_distances(cell)
        best, best_d = None, 1e18
        for (ix, iy), d in dist.items():
            if not self.visited[iy, ix] and d < best_d:
                best, best_d = (ix, iy), d
        if best is None:
            return None
        path = self.astar(cell, best)
        return [self.to_world(ix, iy) for ix, iy in path[::2]] or None

    def nearest_free(self, cell):
        for r in range(1, 40):
            for iy in range(cell[1] - r, cell[1] + r + 1):
                for ix in range(cell[0] - r, cell[0] + r + 1):
                    if self.inside(ix, iy) and self.free[iy, ix]:
                        return (ix, iy)
        return None

    # --- control ------------------------------------------------------------
    def publish(self, vx, vy):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y = vx, vy
        self.pub.publish(msg)

    def stop(self):
        self.publish(0.0, 0.0)

    def step(self):
        if self.finished:
            return
        pose = self.pose()
        if pose is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.t0 > self.time_limit:
            self.finish('time limit')
            return
        if not self.started:
            self.started = True
            self.last_progress = now
            self.get_logger().info(f'Start pose {pose[0]:.2f}, {pose[1]:.2f}')

        if self.path:
            tx, ty = self.path[0]
            dx, dy = tx - pose[0], ty - pose[1]
            if math.hypot(dx, dy) < 0.15:
                self.path.pop(0)
                return
            if self.last_pose is not None and math.hypot(
                    pose[0] - self.last_pose[0], pose[1] - self.last_pose[1]) > 0.03:
                self.last_progress = now
            self.last_pose = pose
            if now - self.last_progress > 8.0:
                self.get_logger().warn('No progress, replanning')
                self.path = []
                return
            cy, sy = math.cos(pose[2]), math.sin(pose[2])
            ex = cy * dx + sy * dy
            ey = -sy * dx + cy * dy
            self.publish(max(-self.vmax, min(self.vmax, 1.5 * ex)),
                         max(-self.vmax, min(self.vmax, 1.5 * ey)))
            self.distance += 0.0
            return

        if now - self.observed_at < 0.5:
            return
        self.observed_at = now
        path = self.plan_next(pose)
        if not path:
            self.finish('all reachable cells visited')
            return
        self.path = path
        self.last_progress = now
        self.get_logger().info(
            f'New path: {len(path)} waypoints, {math.hypot(path[-1][0] - pose[0], path[-1][1] - pose[1]):.1f} m')

    def finish(self, reason):
        self.finished = True
        self.stop()
        self.get_logger().info(f'COVERAGE FINISHED: {reason}. Save the map with map_saver_cli.')


def main():
    rclpy.init()
    node = Coverage()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
