import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray

from taller_omni.trajectory import FIGURAS

# Medidas del Clearpath Dingo-O (no las cambies)
R = 0.05      # radio de la rueda [m]
LX = 0.285    # distancia del centro al eje de las ruedas (adelante/atras) [m]
LY = 0.1854   # distancia del centro al centro de la rueda (lateral) [m]

TOPIC_RUEDAS = '/do100_0000/wheel_velocity_controller/commands' #Topicos de redas
FRECUENCIA = 20.0   # Hz


def mundo_a_robot(theta, dx, dy, dtheta):

    """Velocidad en el marco del MUNDO -> velocidad en el marco del ROBOT.
    Escribir las funciones de Vx, Vy y omega definidas en el README.md
    theta: orientacion del robot. dx, dy, dtheta: velocidad deseada en el mundo.
    Retorna (vx, vy, w) en el marco del robot.
    """
    # >>> ESCRIBE AQUI <<<

    raise NotImplementedError('completa mundo_a_robot() en self_controller.py')


def cinematica_inversa(vx, vy, w):

    """Velocidad del robot (vx, vy, w) -> velocidad angular de cada rueda [rad/s].
    Escribir las funciones de cinemática inversa definidas en el README.md
    Retorna (w_fl, w_fr, w_rl, w_rr): delantera izquierda, delantera derecha,
    trasera izquierda y trasera derecha. Usa R, LX y LY.
    """
    # >>> ESCRIBE AQUI <<<
    raise NotImplementedError('completa cinematica_inversa() en self_controller.py')


class SelfController(Node):

    def __init__(self):
        super().__init__('self_controller', parameter_overrides=[Parameter('use_sim_time', value=True)])
        # Definir la figura
        nombre = self.declare_parameter('figura', 'cuadrado').value
        # Confirmar que la figura existe en trajectory.py definida
        if nombre not in FIGURAS:
            raise SystemExit(f'No existe la figura "{nombre}". Opciones: {", ".join(FIGURAS)}')
        self.figura = FIGURAS[nombre]()
        self.t0 = None
        self.listo_desde = None
        # Crear el topic de ruedas definido previamente
        self.pub_ruedas = self.create_publisher(Float64MultiArray, TOPIC_RUEDAS, 10)
        # La referencia va en el marco 'inicio' (donde arranco el robot); graficar
        # la ubica en el mundo y la compara con el camino real y la odometria.
        self.pub_ref = self.create_publisher(PoseStamped, '/taller/referencia', 10)
        self.create_timer(1.0 / FRECUENCIA, self.paso)
        self.get_logger().info(f'Figura "{nombre}": {self.figura.duracion:.1f} s. '
                               'Esperando al simulador ...')

    def paso(self):
        ahora = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            # Esperar el reloj de la simulacion y al controlador de las ruedas;
            # si no, los primeros comandos se pierden.
            if ahora == 0.0 or self.pub_ruedas.get_subscription_count() == 0:
                return
            if self.listo_desde is None:
                self.listo_desde = ahora
            if ahora - self.listo_desde < 1.0:
                return
            self.t0 = ahora
            self.get_logger().info('En marcha.')

        t = ahora - self.t0
        if t > self.figura.duracion:
            self.enviar((0.0, 0.0, 0.0, 0.0))
            self.get_logger().info('Figura terminada.')
            raise SystemExit

        x, y, theta, dx, dy, dtheta = self.figura(t)
        vx, vy, w = mundo_a_robot(theta, dx, dy, dtheta)
        self.enviar(cinematica_inversa(vx, vy, w))

        ref = PoseStamped()
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.header.frame_id = 'inicio'
        ref.pose.position.x, ref.pose.position.y = x, y
        ref.pose.orientation.z, ref.pose.orientation.w = math.sin(theta / 2), math.cos(theta / 2)
        self.pub_ref.publish(ref)

    def enviar(self, ruedas):
        # Publicación de comando a ruedas
        self.pub_ruedas.publish(Float64MultiArray(data=[float(v) for v in ruedas]))


def main():
    rclpy.init()
    nodo = None
    try:
        nodo = SelfController()
        rclpy.spin(nodo)
    except NotImplementedError as e:
        # No se ha realizado la cinematica inversa y directa o presenta errores
        print(f'Falta tu parte: {e}')
    except (KeyboardInterrupt, SystemExit) as e:
        if isinstance(e, SystemExit) and e.code:
            print(e.code)
    finally:
        if nodo is not None:
            nodo.enviar((0.0, 0.0, 0.0, 0.0))
            nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
