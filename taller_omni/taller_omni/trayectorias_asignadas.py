"""Trayectorias del taller por numero de lista (REFERENCIA DEL INSTRUCTOR).

numero n -> forma = ((n-1) % 6) + 1,  modo = 'ABC'[((n-1) // 6) % 3]
Hay 18 combinaciones distintas; desde el 19 se repiten.

Todas empiezan y terminan en el origen, duran DURACION segundos y caben en la
jaula de drones partiendo del punto de inicio del taller (con margen para el
robot girando).
"""
import math

DURACION = 30.0

FORMAS = {
    1: 'Circulo de radio 0.8 m',
    2: 'Elipse de semiejes 1.2 m (x) y 0.7 m (y)',
    3: 'Ocho vertical: 1.2 m de ancho y 1.8 m de alto',
    4: 'Cuadrado de 1.2 m de lado',
    5: 'Triangulo equilatero de 1.4 m de lado',
    6: 'Rombo de diagonales 1.6 m (x) y 1.8 m (y)',
}
MODOS = {
    'A': 'orientacion fija: theta = 0',
    'B': 'una vuelta completa antihoraria: theta = 2*pi*t/T',
    'C': 'oscila +-45 grados: theta = (pi/4)*sin(2*pi*t/T)',
}
VERTICES = {
    4: [(0, 0), (1.2, 0), (1.2, 1.2), (0, 1.2)],
    5: [(0, 0), (1.4, 0), (0.7, 1.4 * math.sqrt(3) / 2)],
    6: [(0, 0), (0.8, 0.9), (0, 1.8), (-0.8, 0.9)],
}


def asignacion(n):
    return ((n - 1) % 6) + 1, 'ABC'[((n - 1) // 6) % 3]


def poligono(vertices, t, T):
    """Recorre el poligono cerrado a velocidad constante (lados iguales)."""
    nlados = len(vertices)
    tramo = T / nlados
    k = min(int(t // tramo), nlados - 1)
    (x0, y0), (x1, y1) = vertices[k], vertices[(k + 1) % nlados]
    u = (t - k * tramo) / tramo
    return x0 + u * (x1 - x0), y0 + u * (y1 - y0), (x1 - x0) / tramo, (y1 - y0) / tramo


def trayectoria(n, t, T=DURACION):
    forma, modo = asignacion(n)
    W = 2 * math.pi / T
    s, c = math.sin(W * t), math.cos(W * t)
    if forma == 1:
        r = 0.8
        x, y, dx, dy = r * s, r * (1 - c), r * W * c, r * W * s
    elif forma == 2:
        a, b = 1.2, 0.7
        x, y, dx, dy = a * s, b * (1 - c), a * W * c, b * W * s
    elif forma == 3:
        x = 0.6 * math.sin(2 * W * t)
        y = 0.9 * (1 - c)
        dx = 0.6 * 2 * W * math.cos(2 * W * t)
        dy = 0.9 * W * s
    else:
        x, y, dx, dy = poligono(VERTICES[forma], t, T)

    if modo == 'A':
        th, dth = 0.0, 0.0
    elif modo == 'B':
        th, dth = W * t, W
    else:
        th, dth = (math.pi / 4) * s, (math.pi / 4) * W * c
    return x, y, th, dx, dy, dth


if __name__ == '__main__':
    for n in range(1, 19):
        f, m = asignacion(n)
        print(f'{n:2d}: forma {f} ({FORMAS[f]}), modo {m} ({MODOS[m]})')
