"""Devuelve el robot a la posicion inicial del taller (centro-abajo de la jaula).

Uso: ros2 run taller_omni reiniciar   (antes de cada intento)
"""
import sys

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node

INICIO = (-4.875, -0.3)


def main():
    req = Pose()
    req.name = 'do100_0000/robot'
    req.position.x, req.position.y, req.position.z = INICIO[0], INICIO[1], 0.02
    req.orientation.w = 1.0
    ok, rep = Node().request('/world/coral/set_pose', req, Pose, Boolean, 3000)
    if ok and rep.data:
        print(f'Robot en x={INICIO[0]} y={INICIO[1]}, mirando hacia +x.')
    else:
        print('No se pudo mover el robot. Esta corriendo el simulador?')
        sys.exit(1)
