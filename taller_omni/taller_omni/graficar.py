"""Graba la referencia y la pose real, y al terminar guarda un grafico.

Uso: ejecutalo ANTES del nodo del nivel. Se cierra solo cuando la
trayectoria termina (o con Ctrl+C) y guarda la imagen en ~/taller_resultados/.
"""
import math
import os
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

CARPETA = os.path.expanduser('~/taller_resultados')
JAULA = (-6.875, -2.875, -1.225, 2.575)   # zona de drones, donde corre el taller


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Graficar(Node):

    def __init__(self):
        super().__init__('graficar', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.ref = []     # (t, x, y)
        self.real = []    # (t, x, y)
        self.ultima_ref = None
        self.create_subscription(PoseStamped, '/taller/referencia', self.al_recibir_ref, 10)
        self.create_subscription(Odometry, '/taller/pose_real', self.al_recibir_real, 10)
        self.create_timer(0.5, self.revisar_fin)
        self.get_logger().info('Grabando... lanza ahora el nodo del nivel.')

    def al_recibir_ref(self, msg):
        self.ref.append((segundos(msg.header.stamp), msg.pose.position.x, msg.pose.position.y))
        self.ultima_ref = time.monotonic()

    def al_recibir_real(self, msg):
        if self.ref:   # solo mientras corre una trayectoria
            p = msg.pose.pose.position
            self.real.append((segundos(msg.header.stamp), p.x, p.y))

    def revisar_fin(self):
        # La trayectoria termino si ya no llegan referencias
        if self.ultima_ref is not None and time.monotonic() - self.ultima_ref > 2.0:
            raise SystemExit

    def guardar(self):
        if len(self.ref) < 2 or len(self.real) < 2:
            self.get_logger().warn('No hay datos suficientes para graficar.')
            return
        # Error: distancia entre la referencia y la pose real mas cercana en el tiempo
        errores, j = [], 0
        for t, x, y in self.ref:
            while j + 1 < len(self.real) and abs(self.real[j + 1][0] - t) <= abs(self.real[j][0] - t):
                j += 1
            errores.append((t - self.ref[0][0], math.hypot(x - self.real[j][1], y - self.real[j][2])))
        rms = math.sqrt(sum(e * e for _, e in errores) / len(errores))

        fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
        jx0, jx1, jy0, jy1 = JAULA
        a1.plot([jx0, jx1, jx1, jx0, jx0], [jy0, jy0, jy1, jy1, jy0], color='0.6', lw=1,
                label='jaula de drones')
        a1.plot([r[1] for r in self.ref], [r[2] for r in self.ref], '--', lw=2, label='deseada')
        a1.plot([r[1] for r in self.real], [r[2] for r in self.real], lw=2, label='real')
        a1.plot(self.ref[0][1], self.ref[0][2], 'ko', label='inicio')
        a1.set_aspect('equal')
        a1.set_xlabel('x [m]')
        a1.set_ylabel('y [m]')
        a1.set_title('Trayectoria')
        a1.legend(loc='best', fontsize=8)
        a1.grid(alpha=0.3)
        a2.plot([e[0] for e in errores], [e[1] for e in errores])
        a2.set_xlabel('t [s]')
        a2.set_ylabel('error de posicion [m]')
        a2.set_title(f'Error (RMS = {rms:.3f} m, max = {max(e for _, e in errores):.3f} m)')
        a2.grid(alpha=0.3)
        fig.tight_layout()

        os.makedirs(CARPETA, exist_ok=True)
        ruta = os.path.join(CARPETA, time.strftime('trayectoria_%Y%m%d_%H%M%S.png'))
        fig.savefig(ruta, dpi=110)
        self.get_logger().info(f'Error RMS = {rms:.3f} m. Grafico guardado en {ruta}')


def main():
    rclpy.init()
    nodo = Graficar()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        nodo.guardar()
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
