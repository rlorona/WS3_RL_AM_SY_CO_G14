"""
=====================================================================
  TALLER ROBOT OMNIDIRECCIONAL - SOLUCION DEL INSTRUCTOR (no repartir)
=====================================================================

Los nodos de ROS 2 (nivel1, nivel2, nivel3) ya estan escritos y llaman
a las funciones de este archivo. Tu trabajo es completar las funciones
marcadas con  >>> ESCRIBE AQUI <<<  usando el modelo matematico.

Convencion (la misma de ROS):
  - Marco del MUNDO: x, y fijos en el suelo, theta medido desde el eje x.
  - Marco del ROBOT: x hacia adelante, y hacia la izquierda.
  - Angulos en radianes, distancias en metros, tiempo en segundos.

Despues de guardar no hace falta recompilar (el workspace se compilo con
--symlink-install). Solo vuelve a ejecutar el nodo.
"""
import math
import os

try:
    from taller_omni.trayectorias_asignadas import trayectoria as trayectoria_n
except ImportError:
    from trayectorias_asignadas import trayectoria as trayectoria_n

# Numero de lista a resolver (o variable de entorno TALLER_N)
NUMERO = int(os.environ.get('TALLER_N', 1))

# ---------------------------------------------------------------------
# Medidas del Clearpath Dingo-O (no las cambies)
# ---------------------------------------------------------------------
R = 0.05      # radio de la rueda [m]
LX = 0.285    # distancia del centro del robot al eje de las ruedas (adelante/atras) [m]
LY = 0.1854   # distancia del centro del robot al centro de la rueda (lateral) [m]

# Duracion de la trayectoria [s] (la misma para todos)
DURACION = 30.0


# ---------------------------------------------------------------------
# 1) TRAYECTORIA DESEADA (en el marco del mundo, empezando en el origen)
# ---------------------------------------------------------------------
def trayectoria_ejemplo(t):
    """EJEMPLO YA RESUELTO (no es tu trayectoria): circulo de radio 1 m que
    empieza en el origen, mientras el robot da una vuelta sobre si mismo.

    Sirve para probar que todo funciona antes de escribir la tuya:
        ros2 run taller_omni nivel1 --ros-args -p ejemplo:=true
    """
    radio = 1.0
    w = 2 * math.pi / DURACION         # una vuelta en DURACION segundos

    x = radio * math.sin(w * t)
    y = radio * (1 - math.cos(w * t))
    theta = w * t

    dx = radio * w * math.cos(w * t)
    dy = radio * w * math.sin(w * t)
    dtheta = w

    return x, y, theta, dx, dy, dtheta


def trayectoria(t):
    """TU trayectoria: la que te toca segun tu numero de lista (ver la tabla
    del README del paquete o del notebook): una FORMA y un MODO de orientacion.

    Debe empezar en el origen (x=y=theta=0 en t=0), terminar en el origen en
    t=DURACION y devolver (x, y, theta, dx, dy, dtheta), donde dx, dy, dtheta
    son las derivadas respecto a t.
    """
    return trayectoria_n(NUMERO, t, DURACION)


# ---------------------------------------------------------------------
# 2) NIVEL 1 - Del marco del mundo al marco del robot
# ---------------------------------------------------------------------
def mundo_a_robot(theta, dx, dy, dtheta):
    """Convierte la velocidad deseada en el MUNDO a la velocidad que entiende
    el ROBOT (que es lo que recibe cmd_vel).

    Entradas: theta (orientacion actual del robot), dx, dy, dtheta (mundo).
    Retorna (vx, vy, w) en el marco del robot.

    Pista: [vx, vy] = R(theta)^T [dx, dy]   y   w = dtheta
    """
    c, s = math.cos(theta), math.sin(theta)
    vx = c * dx + s * dy
    vy = -s * dx + c * dy
    return vx, vy, dtheta


# ---------------------------------------------------------------------
# 3) NIVEL 2 - Cinematica inversa de las ruedas mecanum
# ---------------------------------------------------------------------
def cinematica_inversa(vx, vy, w):
    """Calcula la velocidad angular de cada rueda [rad/s] para lograr
    (vx, vy, w) en el marco del robot.

    Retorna (w_fl, w_fr, w_rl, w_rr) en ese orden:
      fl = delantera izquierda, fr = delantera derecha,
      rl = trasera izquierda,   rr = trasera derecha.

    Usa R, LX y LY de arriba.
    """
    k = LX + LY
    w_fl = (vx - vy - k * w) / R
    w_fr = (vx + vy + k * w) / R
    w_rl = (vx + vy - k * w) / R
    w_rr = (vx - vy + k * w) / R
    return w_fl, w_fr, w_rl, w_rr


# ---------------------------------------------------------------------
# 4) NIVEL 3 - Control en lazo cerrado
# ---------------------------------------------------------------------
KP = 1.0      # ganancia proporcional: prueba 0, 0.5, 1, 3 y compara


def controlador(deseado, real):
    """Corrige la velocidad deseada usando el error de posicion.

    deseado = (x, y, theta, dx, dy, dtheta)   salida de trayectoria(t)
    real    = (x, y, theta)                    pose medida del robot

    Retorna (dx, dy, dtheta) corregidas, en el marco del MUNDO.

    Pista: velocidad = velocidad_deseada + KP * (posicion_deseada - posicion_real)
    Ojo con el error de angulo: normalizalo a [-pi, pi].
    """
    xd, yd, thd, dxd, dyd, dthd = deseado
    x, y, th = real
    e_th = math.atan2(math.sin(thd - th), math.cos(thd - th))
    return dxd + KP * (xd - x), dyd + KP * (yd - y), dthd + KP * e_th
