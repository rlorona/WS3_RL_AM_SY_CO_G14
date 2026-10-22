"""Nivel 3: lazo cerrado. Igual que el nivel 2, pero corrige con la pose real.

Requiere lanzar el simulador con nivel:=3. Tu parte: modelo.controlador()
(ademas de las funciones de los niveles 1 y 2).
"""
from taller_omni import modelo
from taller_omni.nivel2 import Nivel2
from taller_omni.nodo_base import ejecutar


class Nivel3(Nivel2):

    def __init__(self):
        super().__init__('nivel3')

    def comandar(self, t, deseado, real):
        dx, dy, dtheta = modelo.controlador(deseado, real)
        # Ahora si se usa la orientacion REAL del robot
        vx, vy, w = modelo.mundo_a_robot(real[2], dx, dy, dtheta)
        self.enviar(modelo.cinematica_inversa(vx, vy, w))


def main():
    ejecutar(Nivel3)
