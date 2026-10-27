"""Mide cada intento: trayectoria deseada, pose real y odometria, en un solo nodo.

Lo lanza taller.launch.py. Publica, en el marco 'world' de Gazebo:
  /taller/pose_real            pose exacta del robot en Gazebo (Odometry)
  /taller/odometria            odometria calculada con la velocidad de las ruedas (Odometry)
  /taller/camino/deseado       trayectoria deseada del intento (Path)
  /taller/camino/real          camino real del intento (Path)
  /taller/camino/odometria     camino segun la odometria (Path)
la transformada world -> odom, para ver todo junto con el robot en RViz, y
/do100_0000/platform/odom (odometria de las ruedas para el EKF de Clearpath), que
con el controlador propio nadie mas publica.

Un intento empieza cuando llega la primera referencia en /taller/referencia (en el
marco 'inicio', donde arranco el robot) y termina cuando dejan de llegar. Al terminar guarda un grafico en ~/taller_resultados/.
"""
import math
import os
import threading
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from gz.msgs10.empty_pb2 import Empty  # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: E402
from gz.msgs10.stringmsg_v_pb2 import StringMsg_V  # noqa: E402
from gz.transport13 import Node as GzNode  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

ROBOT = '/do100_0000'
MODELO_GZ = 'do100_0000/robot'
CARPETA = os.path.expanduser('~/taller_resultados')

# Medidas del Dingo-O, para la cinematica directa
R, LX, LY = 0.05, 0.285, 0.1854
RUEDAS = ('front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint')


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def integrar(pose, vx, vy, w, dt):
    """Avanza la pose (x, y, theta) con la velocidad del robot (vx, vy, w) durante dt."""
    x, y, th = pose
    return (x + (math.cos(th) * vx - math.sin(th) * vy) * dt,
            y + (math.sin(th) * vx + math.cos(th) * vy) * dt,
            th + w * dt)


def pose_stamped(stamp, x, y, th):
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = 'world'
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z, msg.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return msg


class Graficar(Node):

    def __init__(self):
        super().__init__('graficar', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.Lock()
        self.pose_gz = None                 # (x, y, theta) en Gazebo, la escribe gz-transport
        self.odom = None                    # (x, y, theta) integrada con las ruedas
        self.t_ruedas = None
        self.world_odom = None
        self.ekf = None                     # ultima pose del EKF de Clearpath (marco odom)
        self.odom_plataforma = None         # (x, y, theta) en el marco odom, para el EKF
        self.clearpath_activo = True        # controlador mecanum de Clearpath
        self.intento = None                 # listas del intento en curso
        self.ultima_ref = None

        self.pub_real = self.create_publisher(Odometry, '/taller/pose_real', 10)
        self.pub_odom = self.create_publisher(Odometry, '/taller/odometria', 10)
        # Con el controlador propio, el de Clearpath esta apagado y nadie publica la
        # odometria de las ruedas que usa su EKF: sin ella, el EKF solo integra la IMU
        # y odom -> base_link se desplaza aunque el robot este quieto.
        self.pub_plataforma = self.create_publisher(Odometry, f'{ROBOT}/platform/odom', 10)
        self.cli_controladores = self.create_client(ListControllers, f'{ROBOT}/controller_manager/list_controllers')
        self.timer_controladores = self.create_timer(0.5, self.revisar_controladores)
        self.pub_caminos = {k: self.create_publisher(Path, f'/taller/camino/{k}', 10)
                            for k in ('deseado', 'real', 'odometria')}
        # El TF del robot va namespaced: world -> odom se agrega en /do100_0000/tf_static
        self.pub_tf_static = self.create_publisher(TFMessage, f'{ROBOT}/tf_static', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.create_subscription(PoseStamped, '/taller/referencia', self.al_recibir_ref, 10)
        self.create_subscription(JointState, f'{ROBOT}/platform/joint_states', self.al_recibir_ruedas, 50)
        self.create_subscription(Odometry, f'{ROBOT}/platform/odom/filtered', self.al_recibir_ekf, 10)
        self.create_timer(0.05, self.publicar)
        self.create_timer(0.5, self.revisar_fin)

        self.gz = GzNode()
        self.conectar_gazebo()

    # --- pose real (Gazebo) ------------------------------------------------
    def conectar_gazebo(self):
        while rclpy.ok():
            ok, rep = self.gz.request('/gazebo/worlds', Empty(), Empty, StringMsg_V, 3000)
            if ok and rep.data:
                break
            self.get_logger().info('Esperando a Gazebo ...')
        topic = f'/world/{rep.data[0]}/dynamic_pose/info'
        if not self.gz.subscribe(Pose_V, topic, self.al_recibir_gz):
            self.get_logger().error(f'No se pudo suscribir a {topic}')

    def al_recibir_gz(self, msg):
        for p in msg.pose:
            if p.name == MODELO_GZ:
                pose = (p.position.x, p.position.y, yaw(p.orientation))
                if math.isfinite(pose[2]):
                    with self.lock:
                        self.pose_gz = pose
                return

    # --- odometria con las ruedas (cinematica directa) ---------------------
    def al_recibir_ruedas(self, msg):
        t = segundos(msg.header.stamp)
        try:
            w_fl, w_fr, w_rl, w_rr = (msg.velocity[msg.name.index(n)] for n in RUEDAS)
        except (ValueError, IndexError):
            return
        if self.odom is None:
            with self.lock:
                self.odom = self.pose_gz      # arranca en la pose real
            return
        if self.t_ruedas is None:
            self.t_ruedas = t
            return
        dt, self.t_ruedas = t - self.t_ruedas, t
        if not 0 < dt < 0.5:
            return
        vx = R / 4 * (w_fl + w_fr + w_rl + w_rr)
        vy = R / 4 * (-w_fl + w_fr + w_rl - w_rr)
        w = R / (4 * (LX + LY)) * (-w_fl + w_fr - w_rl + w_rr)
        self.odom = integrar(self.odom, vx, vy, w, dt)
        self.publicar_plataforma(msg.header.stamp, vx, vy, w, dt)

    def publicar_plataforma(self, stamp, vx, vy, w, dt):
        if self.clearpath_activo or self.ekf is None or self.pose_gz is None:
            return
        if self.odom_plataforma is None:
            # Relevo: continua donde quedo el EKF y vuelve a calcular world -> odom con
            # el robot quieto, por si la odometria salto al cambiar de controlador
            self.odom_plataforma = self.ekf
            self.publicar_world_odom(self.ekf, stamp)
            self.get_logger().info('Publicando la odometria de las ruedas para el EKF de Clearpath')
        self.odom_plataforma = integrar(self.odom_plataforma, vx, vy, w, dt)
        x, y, th = self.odom_plataforma
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
        msg.pose.pose = pose_stamped(stamp, x, y, th).pose
        msg.twist.twist.linear.x, msg.twist.twist.linear.y = vx, vy
        msg.twist.twist.angular.z = w
        for i in range(6):
            msg.pose.covariance[7 * i] = 0.001
            msg.twist.covariance[7 * i] = 0.001
        self.pub_plataforma.publish(msg)

    def revisar_controladores(self):
        # Pregunta al controller_manager si el controlador de Clearpath sigue activo
        if not self.cli_controladores.service_is_ready():
            return
        def respuesta(futuro):
            estados = {c.name: c.state for c in futuro.result().controller}
            if estados.get('platform_velocity_controller', 'active') != 'active':
                self.clearpath_activo = False
                self.timer_controladores.cancel()
        self.cli_controladores.call_async(ListControllers.Request()).add_done_callback(respuesta)

    def al_recibir_ekf(self, msg):
        p = msg.pose.pose
        xo, yo, tho = p.position.x, p.position.y, yaw(p.orientation)
        self.ekf = (xo, yo, tho)
        # Al arrancar, con el robot quieto
        if self.world_odom is None and self.pose_gz is not None:
            self.publicar_world_odom(self.ekf, msg.header.stamp)

    def publicar_world_odom(self, ekf, stamp):
        """world -> odom = pose_gz * odom_ekf^-1 (el robot debe estar quieto)."""
        xo, yo, tho = ekf
        with self.lock:
            xg, yg, thg = self.pose_gz
        th = thg - tho
        x = xg - (math.cos(th) * xo - math.sin(th) * yo)
        y = yg - (math.sin(th) * xo + math.cos(th) * yo)
        self.world_odom = (x, y, th)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id, tf.child_frame_id = 'world', 'odom'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub_tf_static.publish(TFMessage(transforms=[tf]))

    # --- intentos -----------------------------------------------------------
    def al_recibir_ref(self, msg):
        t = segundos(msg.header.stamp)
        with self.lock:
            real = self.pose_gz
        if real is None:
            return
        if self.intento is None:
            # Empieza un intento: la referencia viene en el marco 'inicio' (donde
            # arranco el robot) y la odometria arranca en la pose real
            self.inicio = real
            self.odom = real
            self.t_ruedas = None
            self.intento = {'deseado': [], 'real': [], 'odometria': []}
            self.caminos = {k: Path() for k in self.intento}
            self.get_logger().info('Intento en curso ...')
        x0, y0, th0 = self.inicio
        xi, yi, thi = msg.pose.position.x, msg.pose.position.y, yaw(msg.pose.orientation)
        deseado = (x0 + math.cos(th0) * xi - math.sin(th0) * yi,
                   y0 + math.sin(th0) * xi + math.cos(th0) * yi, th0 + thi)
        ref = (t, deseado[0], deseado[1])
        self.intento['deseado'].append(ref)
        self.intento['real'].append((t, real[0], real[1]))
        self.intento['odometria'].append((t, self.odom[0], self.odom[1]))
        for k, (x, y, th) in (('deseado', deseado),
                              ('real', real), ('odometria', self.odom)):
            self.caminos[k].poses.append(pose_stamped(msg.header.stamp, x, y, th))
        self.ultima_ref = time.monotonic()

    def revisar_fin(self):
        if self.intento is not None and time.monotonic() - self.ultima_ref > 2.0:
            self.guardar(self.intento)
            self.intento = None

    # --- publicaciones --------------------------------------------------------
    def publicar(self):
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            real = self.pose_gz
        for pub, pose in ((self.pub_real, real), (self.pub_odom, self.odom)):
            if pose is None:
                continue
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id, msg.child_frame_id = 'world', 'base_link'
            p = pose_stamped(stamp, *pose).pose
            msg.pose.pose = p
            pub.publish(msg)
        if self.intento is not None:
            for k, camino in self.caminos.items():
                camino.header.stamp, camino.header.frame_id = stamp, 'world'
                self.pub_caminos[k].publish(camino)

    # --- grafico ----------------------------------------------------------------
    def guardar(self, intento):
        ref = intento['deseado']
        if len(ref) < 2:
            return
        fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
        a1.plot([r[1] for r in ref], [r[2] for r in ref], 'k--', lw=2, label='deseada')
        resumen = []
        for k, color in (('real', 'C0'), ('odometria', 'C1')):
            puntos = intento[k]
            errores = [math.hypot(r[1] - p[1], r[2] - p[2]) for r, p in zip(ref, puntos)]
            rms = math.sqrt(sum(e * e for e in errores) / len(errores))
            resumen.append(f'{k}: RMS {rms:.3f} m')
            a1.plot([p[1] for p in puntos], [p[2] for p in puntos], color=color, lw=2, label=k)
            a2.plot([r[0] - ref[0][0] for r in ref], errores, color=color,
                    label=f'{k} (RMS {rms:.3f} m, max {max(errores):.3f} m)')
        a1.plot(ref[0][1], ref[0][2], 'ko', label='inicio')
        a1.set_aspect('equal'); a1.set_xlabel('x [m]'); a1.set_ylabel('y [m]')
        a1.set_title('Trayectoria'); a1.legend(loc='best', fontsize=8); a1.grid(alpha=0.3)
        a2.set_xlabel('t [s]'); a2.set_ylabel('error respecto a la deseada [m]')
        a2.set_title('Error de posición'); a2.legend(fontsize=8); a2.grid(alpha=0.3)
        fig.tight_layout()
        os.makedirs(CARPETA, exist_ok=True)
        ruta = os.path.join(CARPETA, time.strftime('trayectoria_%Y%m%d_%H%M%S.png'))
        fig.savefig(ruta, dpi=110)
        plt.close(fig)
        self.get_logger().info(f'{", ".join(resumen)}. Grafico guardado en {ruta}')


def main():
    rclpy.init()
    nodo = Graficar()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        if nodo.intento is not None:
            nodo.guardar(nodo.intento)
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
