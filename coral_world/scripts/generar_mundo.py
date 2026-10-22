#!/usr/bin/env python3
"""Genera worlds/coral.sdf a partir del plano CoRAL (PL-002, escala 1:50).

Coordenadas en metros extraidas del PDF vectorial. Origen en el centro del
laboratorio, +X hacia la derecha del plano y +Y hacia arriba del plano.
Todo es geometria primitiva (box) en un unico modelo estatico para que la
simulacion sea liviana.

Uso: python3 generar_mundo.py [salida.sdf]
"""
import os
import random
import sys

from PIL import Image, ImageDraw

# --- Parametros generales -------------------------------------------------
H_MURO = 2.5          # altura de muros y columnas
E_MURO = 0.15         # espesor de los muros exteriores (hacia afuera)
H_PUERTA = 2.05
H_MESA = 0.75
H_MUEBLE = 0.9
H_ESCALON = 0.15      # tarima de la sala derecha
H_JAULA = 2.5
JAULA_ENTRADA_Y = (1.10, 2.10)   # abertura en el lado derecho de la jaula
METER_SILLA = 0.35    # cuanto se mete cada silla bajo su mesa respecto al plano
BALDOSA = 0.60        # lado de la baldosa del piso

# Limites interiores del laboratorio
X0, X1, Y0, Y1 = -7.675, 7.675, -3.375, 3.375

C = {
    'muro': (0.85, 0.85, 0.82),
    'columna': (0.75, 0.75, 0.72),
    'puerta': (0.55, 0.38, 0.22),
    'piso': (0.45, 0.45, 0.43),   # exterior del laboratorio
    'tarima': (0.70, 0.60, 0.45),
    'mesa': (0.80, 0.68, 0.50),
    'patas': (0.25, 0.25, 0.25),
    'silla': (0.07, 0.07, 0.07),
    'impresora': (0.20, 0.20, 0.22),
    'red': (0.10, 0.10, 0.10),
    'poste': (0.30, 0.30, 0.30),
}

partes = []


def box(nombre, x0, x1, y0, y1, z0, z1, color, alpha=1.0, colision=True):
    """Caja alineada a los ejes definida por sus limites."""
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
    sx, sy, sz = abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)
    r, g, b = C[color]
    pose = f'{cx:.4f} {cy:.4f} {cz:.4f} 0 0 0'
    geo = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    xml = ''
    if colision:
        xml += f'      <collision name="{nombre}_c"><pose>{pose}</pose>{geo}</collision>\n'
    mat = (f'<material><ambient>{r} {g} {b} {alpha}</ambient>'
           f'<diffuse>{r} {g} {b} {alpha}</diffuse></material>')
    trans = f'<transparency>{1 - alpha:.2f}</transparency>' if alpha < 1 else ''
    xml += f'      <visual name="{nombre}_v"><pose>{pose}</pose>{geo}{mat}{trans}</visual>\n'
    partes.append(xml)


def pata(nombre, x, y, z, h, lado=0.05, m=0.03):
    """Pata cuadrada en una esquina del tablero. x = (coordenada, dx) e
    y = (coordenada, dy), con dx/dy = +1/-1 apuntando hacia el interior."""
    (px, dx), (py, dy) = x, y
    x0 = px + dx * m
    y0 = py + dy * m
    box(nombre, min(x0, x0 + dx * lado), max(x0, x0 + dx * lado),
        min(y0, y0 + dy * lado), max(y0, y0 + dy * lado), z, z + h - 0.03, 'patas')


def mesa(nombre, x0, x1, y0, y1, z=0.0, h=H_MESA):
    """Modelo unico de mesa/escritorio: tablero de madera y 4 patas."""
    box(f'{nombre}_tablero', x0, x1, y0, y1, z + h - 0.03, z + h, 'mesa')
    for i, (ex, ey) in enumerate([((x0, 1), (y0, 1)), ((x1, -1), (y0, 1)),
                                  ((x0, 1), (y1, -1)), ((x1, -1), (y1, -1))]):
        pata(f'{nombre}_pata{i}', ex, ey, z, h)


def mesa_l(nombre, tramos, esquinas, z=0.0, h=H_MESA):
    """Escritorio en L: el tablero son dos rectangulos y las patas van solo en
    las esquinas exteriores (la esquina interior queda libre para la silla)."""
    for i, (x0, x1, y0, y1) in enumerate(tramos):
        box(f'{nombre}_tablero{i}', x0, x1, y0, y1, z + h - 0.03, z + h, 'mesa')
    for i, (ex, ey) in enumerate(esquinas):
        pata(f'{nombre}_pata{i}', ex, ey, z, h)


def silla(nombre, x0, x1, y0, y1, respaldo, z=0.0, meter=METER_SILLA):
    """Silla 0.45x0.45: patas, asiento y respaldo en el lado indicado (N/S/E/O).
    (x0, x1, y0, y1) es la posicion del plano; la silla se desplaza 'meter'
    metros hacia su mesa (lado opuesto al respaldo) para quedar metida."""
    dx, dy = {'O': (meter, 0), 'E': (-meter, 0), 'S': (0, meter), 'N': (0, -meter)}[respaldo]
    x0, x1, y0, y1 = x0 + dx, x1 + dx, y0 + dy, y1 + dy
    pata = 0.03
    for i, (px, py) in enumerate([(x0, y0), (x1 - pata, y0), (x0, y1 - pata), (x1 - pata, y1 - pata)]):
        box(f'{nombre}_pata{i}', px, px + pata, py, py + pata, z, z + 0.42, 'silla')
    box(f'{nombre}_asiento', x0, x1, y0, y1, z + 0.42, z + 0.47, 'silla')
    e = 0.05
    rx0, rx1, ry0, ry1 = {
        'O': (x0, x0 + e, y0, y1), 'E': (x1 - e, x1, y0, y1),
        'S': (x0, x1, y0, y0 + e), 'N': (x0, x1, y1 - e, y1),
    }[respaldo]
    box(f'{nombre}_respaldo', rx0, rx1, ry0, ry1, z + 0.47, z + 0.90, 'silla')


# --- Muros exteriores (el plano marca la cara interior) ---------------------
e = E_MURO
box('muro_norte', X0 - e, X1 + e, Y1, Y1 + e, 0, H_MURO, 'muro')
box('muro_oeste', X0 - e, X0, Y0 - e, Y1, 0, H_MURO, 'muro')
box('muro_este', X1, X1 + e, Y0 - e, Y1, 0, H_MURO, 'muro')
# Muro sur con la puerta de entrada (cerrada) entre x=-7.175 y x=-6.275
box('muro_sur_a', X0, -7.175, Y0 - e, Y0, 0, H_MURO, 'muro')
box('muro_sur_b', -6.275, X1, Y0 - e, Y0, 0, H_MURO, 'muro')
box('puerta_entrada', -7.175, -6.275, Y0 - e, Y0 - e + 0.05, 0, H_PUERTA, 'puerta')
box('dintel_entrada', -7.175, -6.275, Y0 - e, Y0, H_PUERTA, H_MURO, 'muro')

# Columnas
box('columna_no', -7.675, -7.275, 2.975, 3.375, 0, H_MURO, 'columna')
box('columna_so', -7.675, -7.275, -3.375, -2.975, 0, H_MURO, 'columna')
box('columna_n', 1.625, 2.255, 3.175, 3.375, 0, H_MURO, 'columna')
box('columna_s', 1.625, 2.255, -3.375, -2.975, 0, H_MURO, 'columna')

# Tabique entre salas (x 2.155-2.255) con puerta abierta de 0.9 m
box('tabique_n', 2.155, 2.255, -1.575, 3.175, 0, H_MURO, 'muro')
box('tabique_s', 2.155, 2.255, -2.975, -2.475, 0, H_MURO, 'muro')
box('tabique_dintel', 2.155, 2.255, -2.475, -1.575, H_PUERTA, H_MURO, 'muro')
box('puerta_interior', 2.255, 3.155, -2.475, -2.435, 0, H_PUERTA, 'puerta')

# --- Sala principal -----------------------------------------------------------
# Jaula de drones (red) con entrada en el lado derecho
jx0, jx1, jy0, jy1 = -6.875, -2.875, -1.225, 2.575
t = 0.02
box('jaula_red_s', jx0, jx1, jy0, jy0 + t, 0, H_JAULA, 'red', alpha=0.35)
box('jaula_red_n', jx0, jx1, jy1 - t, jy1, 0, H_JAULA, 'red', alpha=0.35)
box('jaula_red_o', jx0, jx0 + t, jy0, jy1, 0, H_JAULA, 'red', alpha=0.35)
box('jaula_red_e1', jx1 - t, jx1, jy0, JAULA_ENTRADA_Y[0], 0, H_JAULA, 'red', alpha=0.35)
box('jaula_red_e2', jx1 - t, jx1, JAULA_ENTRADA_Y[1], jy1, 0, H_JAULA, 'red', alpha=0.35)
for i, (px, py) in enumerate([(jx0, jy0), (jx1, jy0), (jx0, jy1), (jx1, jy1),
                              (jx1, JAULA_ENTRADA_Y[0]), (jx1, JAULA_ENTRADA_Y[1])]):
    box(f'jaula_poste{i}', px - 0.03, px + 0.03, py - 0.03, py + 0.03, 0, H_JAULA, 'poste')

# Escritorio en L (L1, PC experimentos) y su silla
mesa_l('escritorio_l1',
       [(-2.675, -2.075, 2.175, 3.375), (-2.075, -1.475, 2.775, 3.375)],
       [((-2.675, 1), (2.175, 1)), ((-2.075, -1), (2.175, 1)), ((-1.475, -1), (2.775, 1)),
        ((-1.475, -1), (3.375, -1)), ((-2.675, 1), (3.375, -1))])
# Centrada en el tramo libre para no chocar con la pata de la esquina
silla('silla_l1', -2.020, -1.570, 2.225, 2.675, 'S')

# Mesa de impresoras 3D (mismo modelo que las mesas centrales)
mesa('mesa_impresoras', -0.375, 1.525, 2.475, 3.375)
for i, x in enumerate([-0.275, 0.325, 0.925]):
    box(f'impresora{i}', x, x + 0.5, 2.675, 3.175, H_MESA, H_MESA + 0.5, 'impresora')

# Mesas centrales y sus sillas
mesa('mesa_cp2', -1.845, -0.945, -1.050, 0.850)
mesa('mesa_cp1', 0.605, 1.505, -1.050, 0.850)
for nombre, x0, resp in [('a', -2.395, 'O'), ('b', -0.845, 'E'), ('c', 0.055, 'O'), ('d', 1.605, 'E')]:
    silla(f'silla_{nombre}1', x0, x0 + 0.45, -0.800, -0.350, resp)
    silla(f'silla_{nombre}2', x0, x0 + 0.45, 0.150, 0.600, resp)

# Mesitas del muro inferior (C4) y sillas
for i, x in enumerate([-5.875, -4.375, -2.875, -1.375, 0.125]):
    mesa(f'mesita{i}', x, x + 1.5, -3.375, -2.775)
for i, x in enumerate([-5.350, -3.850, -2.350, -0.850, 0.650]):
    silla(f'silla_mesita{i}', x, x + 0.45, -2.675, -2.225, 'N')

# --- Sala derecha ---------------------------------------------------------------
# Muebles del muro superior (B14-B15)
for i, x in enumerate([2.865, 3.765, 4.665]):
    mesa(f'mueble_n{i}', x + 0.005, x + 0.895, 2.975, 3.375)

# Mesa de reuniones (CP3) con 6 sillas
mesa('mesa_reuniones', 3.615, 4.815, -0.600, 1.400)
silla('silla_r1', 3.065, 3.515, -0.325, 0.125, 'O')
silla('silla_r2', 3.065, 3.515, 0.675, 1.125, 'O')
silla('silla_r3', 4.915, 5.365, -0.325, 0.125, 'E')
silla('silla_r4', 4.915, 5.365, 0.675, 1.125, 'E')
silla('silla_r5', 3.990, 4.440, -1.150, -0.700, 'S')
silla('silla_r6', 3.990, 4.440, 1.500, 1.950, 'N')

# Tarima / escalon (borde: x=6.175 arriba, diagonal, x=5.755 abajo)
z = H_ESCALON
box('tarima_n', 6.175, X1, -0.625, Y1, 0, z, 'tarima')
box('tarima_s', 5.755, X1, Y0, -0.867, 0, z, 'tarima')
box('tarima_diag', 5.965, X1, -0.867, -0.625, 0, z, 'tarima')

# Sobre la tarima: escritorio B16, cafe, escritorio L3 y sillas
mesa('escritorio_b16', 6.175, 6.775, 1.875, 3.375, z=z)
silla('silla_b16', 6.875, 7.325, 2.400, 2.850, 'E', z=z)
mesa('mueble_cafe', 7.175, 7.675, -0.500, 0.500, z=z, h=H_MUEBLE)
mesa_l('escritorio_l3',
       [(5.755, 6.355, -3.375, -1.875), (6.355, 7.255, -3.375, -2.775)],
       [((5.755, 1), (-1.875, -1)), ((6.355, -1), (-1.875, -1)), ((7.255, -1), (-2.775, -1)),
        ((7.255, -1), (-3.375, 1)), ((5.755, 1), (-3.375, 1))], z=z)
silla('silla_l3', 6.505, 6.955, -2.675, -2.225, 'N', z=z)

# --- Piso de baldosas --------------------------------------------------------
def textura_piso(ruta, px_m=80):
    """Baldosas color crema de BALDOSA m, alineadas a la esquina (X0, Y0).
    La imagen cubre exactamente el laboratorio, asi que no se repite."""
    w, h = round((X1 - X0) * px_m), round((Y1 - Y0) * px_m)
    img = Image.new('RGB', (w, h))
    d = ImageDraw.Draw(img)
    rnd = random.Random(7)          # siempre la misma textura
    paso = BALDOSA * px_m
    junta = max(2, round(0.004 * px_m * 2))
    for i in range(int(w / paso) + 1):
        for j in range(int(h / paso) + 1):
            v = rnd.randint(-6, 6)  # leve variacion entre baldosas
            color = (232 + v, 220 + v, 190 + v)
            x0, y0 = round(i * paso), round(h - (j + 1) * paso)  # fila 0 abajo (y = Y0)
            d.rectangle([x0, y0, round((i + 1) * paso) - 1, round(h - j * paso) - 1], fill=color)
            d.rectangle([x0, y0, x0 + junta - 1, round(h - j * paso) - 1], fill=(196, 186, 162))
            d.rectangle([x0, round(h - j * paso) - junta, round((i + 1) * paso) - 1,
                         round(h - j * paso) - 1], fill=(196, 186, 162))
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    img.save(ruta)


# --- Mundo ----------------------------------------------------------------------
pr, pg, pb = C['piso']
sdf = f'''<?xml version="1.0" ?>
<!-- Generado por scripts/generar_mundo.py a partir del plano CoRAL PL-002. No editar a mano. -->
<sdf version="1.9">
  <world name="coral">
    <physics type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <plugin name="gz::sim::systems::Physics" filename="libgz-sim-physics-system.so"/>
    <plugin name="gz::sim::systems::UserCommands" filename="libgz-sim-user-commands-system.so"/>
    <plugin name="gz::sim::systems::SceneBroadcaster" filename="libgz-sim-scene-broadcaster-system.so"/>
    <plugin name="gz::sim::systems::Sensors" filename="libgz-sim-sensors-system.so">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="gz::sim::systems::Imu" filename="libgz-sim-imu-system.so"/>
    <plugin name="gz::sim::systems::NavSat" filename="libgz-sim-navsat-system.so"/>

    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.8 0.9 1</background>
      <shadows>false</shadows>
      <grid>false</grid>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sol">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -1</direction>
    </light>

    <model name="piso">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
        <visual name="v">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>{pr} {pg} {pb} 1</ambient><diffuse>{pr} {pg} {pb} 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="piso_baldosas">
      <static>true</static>
      <link name="link">
        <visual name="v">
          <pose>{(X0 + X1) / 2:.4f} {(Y0 + Y1) / 2:.4f} 0.001 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>{X1 - X0:.3f} {Y1 - Y0:.3f}</size></plane></geometry>
          <material>
            <diffuse>1 1 1 1</diffuse><ambient>1 1 1 1</ambient>
            <pbr><metal>
              <albedo_map>model://coral_world/textures/piso_baldosas.png</albedo_map>
              <roughness>0.7</roughness><metalness>0</metalness>
            </metal></pbr>
          </material>
        </visual>
      </link>
    </model>

    <model name="laboratorio_coral">
      <static>true</static>
      <link name="link">
{''.join(partes)}      </link>
    </model>
  </world>
</sdf>
'''

salida = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'worlds', 'coral.sdf')
os.makedirs(os.path.dirname(os.path.abspath(salida)), exist_ok=True)
textura_piso(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                          'textures', 'piso_baldosas.png'))
with open(salida, 'w') as f:
    f.write(sdf)
print(f'{salida}: {len(partes)} elementos')
