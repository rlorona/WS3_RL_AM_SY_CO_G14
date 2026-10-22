"""Nivel 1: el robot recibe (vx, vy, w) por cmd_vel.

Tu parte: modelo.mundo_a_robot(). La orientacion usada es la DESEADA
(lazo abierto): el robot no mira donde esta realmente.
"""
from geometry_msgs.msg import TwistStamped

from taller_omni import modelo
from taller_omni.nodo_base import ROBOT, NodoTrayectoria, ejecutar


class Nivel1(NodoTrayectoria):

    def __init__(self):
        super().__init__('nivel1')
        self.pub = self.create_publisher(TwistStamped, f'{ROBOT}/cmd_vel', 10)

    def comandar(self, t, deseado, real):
        x, y, theta, dx, dy, dtheta = deseado
        vx, vy, w = modelo.mundo_a_robot(theta, dx, dy, dtheta)
        self.enviar(vx, vy, w)

    def enviar(self, vx, vy, w):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = vx, vy, w
        self.pub.publish(msg)

    def detener(self):
        self.enviar(0.0, 0.0, 0.0)


def main():
    ejecutar(Nivel1)
