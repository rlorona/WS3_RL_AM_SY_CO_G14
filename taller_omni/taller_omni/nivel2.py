"""Nivel 2: el robot recibe directamente la velocidad de cada rueda.

Requiere lanzar el simulador con nivel:=2 (sin el controlador mecanum de
Clearpath). Tu parte: modelo.mundo_a_robot() y modelo.cinematica_inversa().
"""
from std_msgs.msg import Float64MultiArray

from taller_omni import modelo
from taller_omni.nodo_base import ROBOT, NodoTrayectoria, ejecutar

TOPIC_RUEDAS = f'{ROBOT}/wheel_velocity_controller/commands'


class Nivel2(NodoTrayectoria):

    def __init__(self, nombre='nivel2'):
        super().__init__(nombre)
        self.pub = self.create_publisher(Float64MultiArray, TOPIC_RUEDAS, 10)

    def comandar(self, t, deseado, real):
        x, y, theta, dx, dy, dtheta = deseado
        vx, vy, w = modelo.mundo_a_robot(theta, dx, dy, dtheta)
        self.enviar(modelo.cinematica_inversa(vx, vy, w))

    def enviar(self, ruedas):
        # Orden: delantera izq, delantera der, trasera izq, trasera der
        self.pub.publish(Float64MultiArray(data=[float(v) for v in ruedas]))

    def detener(self):
        self.enviar((0.0, 0.0, 0.0, 0.0))


def main():
    ejecutar(Nivel2)
