"""Publica la pose REAL del robot (la de Gazebo) en /taller/pose_real.

Es la "verdad" contra la que se compara la trayectoria; un robot real no la
tendria y usaria odometria/localizacion.
"""
import math
import threading

import rclpy
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


class PoseReal(Node):

    def __init__(self):
        super().__init__('pose_real', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.declare_parameter('mundo', 'coral')
        self.declare_parameter('modelo', 'do100_0000/robot')
        self.modelo = self.get_parameter('modelo').value
        topic = f"/world/{self.get_parameter('mundo').value}/dynamic_pose/info"

        self.lock = threading.Lock()
        self.pose = None
        self.gz = GzNode()
        if not self.gz.subscribe(Pose_V, topic, self.al_recibir_gz):
            self.get_logger().error(f'No se pudo suscribir a {topic}')
        self.pub = self.create_publisher(Odometry, '/taller/pose_real', 10)
        self.create_timer(0.02, self.publicar)

    def al_recibir_gz(self, msg):
        for p in msg.pose:
            if p.name == self.modelo:
                with self.lock:
                    self.pose = p
                return

    def publicar(self):
        with self.lock:
            p = self.pose
        if p is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = p.position.x
        msg.pose.pose.position.y = p.position.y
        msg.pose.pose.position.z = p.position.z
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
        if not math.isfinite(o.w):
            return
        self.pub.publish(msg)


def main():
    rclpy.init()
    nodo = PoseReal()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
