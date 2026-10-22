"""Logica comun de los nodos del taller (no hace falta editar este archivo).

Cada nivel hereda de NodoTrayectoria y solo implementa comandar(), que
recibe la referencia y la pose real y publica el comando al robot.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from taller_omni import modelo

ROBOT = '/do100_0000'
FRECUENCIA = 20.0   # Hz del lazo de control


def yaw_de_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def normalizar(angulo):
    return math.atan2(math.sin(angulo), math.cos(angulo))


class NodoTrayectoria(Node):
    """Recorre modelo.trayectoria(t) una vez, partiendo de la pose actual."""

    def __init__(self, nombre):
        super().__init__(nombre, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.origen = None          # pose del mundo donde arranca la trayectoria
        self.pose_mundo = None
        self.t0 = None
        self.terminado = False
        self.espera_conexion = None
        # -p ejemplo:=true usa la trayectoria resuelta de ejemplo en vez de la tuya
        self.trayectoria = (modelo.trayectoria_ejemplo
                            if self.declare_parameter('ejemplo', False).value
                            else modelo.trayectoria)

        self.create_subscription(Odometry, '/taller/pose_real', self.al_recibir_pose, 10)
        self.pub_ref = self.create_publisher(PoseStamped, '/taller/referencia', 10)
        self.create_timer(1.0 / FRECUENCIA, self.paso)
        self.get_logger().info('Esperando la pose del robot en /taller/pose_real ...')

    # --- pose ------------------------------------------------------------
    def al_recibir_pose(self, msg):
        p = msg.pose.pose
        self.pose_mundo = (p.position.x, p.position.y, yaw_de_quaternion(p.orientation))

    def a_marco_inicial(self, x, y, th):
        """Pose del mundo -> marco con origen en la pose inicial del robot."""
        x0, y0, th0 = self.origen
        c, s = math.cos(th0), math.sin(th0)
        dx, dy = x - x0, y - y0
        return c * dx + s * dy, -s * dx + c * dy, normalizar(th - th0)

    def a_mundo(self, x, y, th):
        x0, y0, th0 = self.origen
        c, s = math.cos(th0), math.sin(th0)
        return x0 + c * x - s * y, y0 + s * x + c * y, th + th0

    # --- lazo ------------------------------------------------------------
    def paso(self):
        if self.terminado or self.pose_mundo is None:
            return
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            if ahora == 0.0:
                return      # todavia no llega /clock
            # Esperar a que el controlador del robot y el graficador esten
            # conectados; si no, los primeros comandos se pierden y el robot
            # arranca tarde respecto a la referencia.
            if self.pub.get_subscription_count() == 0:
                return
            if self.espera_conexion is None:
                self.espera_conexion = ahora
            if ahora - self.espera_conexion < 1.0:
                return
            self.t0 = ahora
            self.origen = self.pose_mundo
            self.get_logger().info(
                f'Inicio en x={self.origen[0]:.2f} y={self.origen[1]:.2f} '
                f'theta={self.origen[2]:.2f}. Duracion {modelo.DURACION:.0f} s')

        t = ahora - self.t0
        if t > modelo.DURACION:
            self.detener()
            self.terminado = True
            self.get_logger().info('Trayectoria terminada. Ctrl+C para salir.')
            return

        real = self.a_marco_inicial(*self.pose_mundo)
        try:
            deseado = self.trayectoria(t)
            self.comandar(t, deseado, real)
        except NotImplementedError as e:
            self.get_logger().error(str(e))
            self.detener()
            self.terminado = True
            raise SystemExit(1)
        self.publicar_referencia(deseado)

    def publicar_referencia(self, deseado):
        x, y, th = self.a_mundo(*deseado[:3])
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub_ref.publish(msg)

    # --- a implementar por cada nivel --------------------------------------
    def comandar(self, t, deseado, real):
        raise NotImplementedError

    def detener(self):
        raise NotImplementedError


def ejecutar(clase):
    rclpy.init()
    nodo = clase()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            nodo.detener()
        except Exception:
            pass
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
