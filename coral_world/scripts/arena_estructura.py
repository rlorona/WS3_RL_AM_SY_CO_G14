"""Propuesta de estructura manufacturable para la jaula (arena) del CoRAL.

Estructura de perfil de aluminio ranurado (40 serie, ranura 8):
  - postes 40x40 en esquinas, a media luz y en el vano de la puerta;
  - anillo superior 40x80 (80 en vertical) que rigidiza y lleva las camaras;
  - riel inferior 40x40 a ras de piso para atar el borde bajo de la red;
  - escuadras a 45 grados (cartelas) en las esquinas superiores.
Red sin nudos negra con relinga (cuerda de borde) atada a canamos M8 roscados
en tuercas-T de la ranura interior, separados RED_PASO. Camaras de tracking
por fuera de la red, colgadas del anillo superior.

Ejecutar dentro de Blender (Scripting > Run Script, o via MCP). Reemplaza la
coleccion 'arena' y oculta la jaula simplificada del mundo (jaula_*).
Imprime la lista de corte y de herrajes. Todas las medidas en metros.
"""
import math
from collections import Counter

import bpy
from mathutils import Matrix, Vector

# --- Parametros ---------------------------------------------------------------
X0, X1, Y0, Y1 = -6.875, -2.875, -1.225, 2.575  # huella exterior (= jaula actual)
PUERTA_Y = (1.10, 2.10)    # vano libre de la puerta, lado este
H = 2.40                   # altura total (tope del anillo superior)
P = 0.040                  # ancho de perfil
H_ANILLO = 0.080           # perfil 40x80 del anillo superior
CARTELA = 0.40             # catetos de la escuadra a 45 grados
RED_OFFSET = 0.05          # distancia de la red a la cara interior del perfil
RED_MALLA = 0.025          # abertura de la malla
RED_HILO = 0.002           # grosor del hilo
RED_PASO = 0.30            # separacion de canamos a lo largo de la relinga
TRASLAPE_PUERTA = 0.15     # la cortina de la puerta cubre esto a cada lado
N_CAMARAS = 8

Z_ANILLO = H - H_ANILLO    # cara inferior del anillo = largo de los postes

corte = Counter()      # (perfil, largo mm, nota) -> cantidad
herrajes = Counter()


# --- Utilidades -----------------------------------------------------------------
def coleccion(nombre, padre=None):
    col = bpy.data.collections.get(nombre)
    if col is None:
        col = bpy.data.collections.new(nombre)
        (padre or bpy.context.scene.collection).children.link(col)
    return col


def material(nombre, color, metal=0.0, rugosidad=0.5):
    mat = bpy.data.materials.get(nombre) or bpy.data.materials.new(nombre)
    mat.use_nodes = True
    mat.diffuse_color = (*color, 1)
    bsdf = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Base Color'].default_value = (*color, 1)
    bsdf.inputs['Metallic'].default_value = metal
    bsdf.inputs['Roughness'].default_value = rugosidad
    return mat


def material_red():
    """Malla cuadrada procedural: hilo negro y el resto transparente."""
    mat = bpy.data.materials.get('red_sin_nudos')
    if mat:
        return mat
    mat = bpy.data.materials.new('red_sin_nudos')
    mat.use_nodes = True
    mat.diffuse_color = (0.02, 0.02, 0.02, 0.35)
    nt = mat.node_tree
    nt.nodes.clear()
    salida = nt.nodes.new('ShaderNodeOutputMaterial')
    mezcla = nt.nodes.new('ShaderNodeMixShader')
    transp = nt.nodes.new('ShaderNodeBsdfTransparent')
    hilo = nt.nodes.new('ShaderNodeBsdfPrincipled')
    hilo.inputs['Base Color'].default_value = (0.02, 0.02, 0.02, 1)
    hilo.inputs['Roughness'].default_value = 0.8
    uv = nt.nodes.new('ShaderNodeTexCoord')
    sep = nt.nodes.new('ShaderNodeSeparateXYZ')
    nt.links.new(uv.outputs['UV'], sep.inputs[0])  # UV en metros

    def es_hilo(salida_eje):
        div = nt.nodes.new('ShaderNodeMath'); div.operation = 'DIVIDE'
        div.inputs[1].default_value = RED_MALLA + RED_HILO
        fr = nt.nodes.new('ShaderNodeMath'); fr.operation = 'FRACT'
        lt = nt.nodes.new('ShaderNodeMath'); lt.operation = 'LESS_THAN'
        lt.inputs[1].default_value = RED_HILO / (RED_MALLA + RED_HILO)
        nt.links.new(salida_eje, div.inputs[0])
        nt.links.new(div.outputs[0], fr.inputs[0])
        nt.links.new(fr.outputs[0], lt.inputs[0])
        return lt.outputs[0]

    mx = nt.nodes.new('ShaderNodeMath'); mx.operation = 'MAXIMUM'
    nt.links.new(es_hilo(sep.outputs['X']), mx.inputs[0])
    nt.links.new(es_hilo(sep.outputs['Y']), mx.inputs[1])
    nt.links.new(mx.outputs[0], mezcla.inputs['Fac'])
    nt.links.new(transp.outputs[0], mezcla.inputs[1])
    nt.links.new(hilo.outputs[0], mezcla.inputs[2])
    nt.links.new(mezcla.outputs[0], salida.inputs['Surface'])
    for attr, val in (('surface_render_method', 'DITHERED'), ('blend_method', 'HASHED')):
        try:
            setattr(mat, attr, val)
        except (AttributeError, TypeError):
            pass
    return mat


def contorno_perfil(w, h, ranura=0.008, labio=0.0045, ancho_int=0.016, prof=0.012):
    """Seccion de perfil ranurado serie 40 (una ranura cada 40 mm por cara)."""
    pts = []
    esquinas = [(-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2)]
    for i in range(4):
        a, b = Vector(esquinas[i]), Vector(esquinas[(i + 1) % 4])
        largo = (b - a).length
        d = (b - a) / largo
        n = Vector((d.y, -d.x))       # normal exterior
        pts.append(a)
        for k in range(int(round(largo / 0.04))):
            c = a + d * (0.02 + 0.04 * k)
            for s, p in ((-ranura / 2, 0), (-ranura / 2, labio), (-ancho_int / 2, labio),
                         (-ancho_int / 2 + 0.003, prof), (ancho_int / 2 - 0.003, prof),
                         (ancho_int / 2, labio), (ranura / 2, labio), (ranura / 2, 0)):
                pts.append(c + d * s - n * p)
    return pts


_mallas_perfil = {}


def malla_perfil(w, h):
    clave = (w, h)
    if clave not in _mallas_perfil:
        sec = contorno_perfil(w, h)
        n = len(sec)
        verts = [(p.x, p.y, 0) for p in sec] + [(p.x, p.y, 1) for p in sec]
        caras = [list(range(n))[::-1], list(range(n, 2 * n))]
        caras += [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
        me = bpy.data.meshes.new(f'perfil_{int(w*1000)}x{int(h*1000)}')
        me.from_pydata(verts, [], caras)
        _mallas_perfil[clave] = me
    return _mallas_perfil[clave]


def perfil(nombre, a, b, w, h, col, mat, rodar=0.0, nota=''):
    """Perfil de seccion w x h desde a hasta b (centros de las caras extremas)."""
    a, b = Vector(a), Vector(b)
    eje = b - a
    largo = eje.length
    rot = eje.to_track_quat('Z', 'Y' if abs(eje.normalized().z) < 0.99 else 'X').to_matrix().to_4x4()
    mw = Matrix.Translation(a) @ rot @ Matrix.Rotation(rodar, 4, 'Z') @ Matrix.Diagonal((1, 1, largo, 1))
    obj = bpy.data.objects.new(nombre, malla_perfil(w, h))
    obj.matrix_world = mw
    if not obj.data.materials:
        obj.data.materials.append(mat)
    col.objects.link(obj)
    corte[(f'{int(w*1000)}x{int(h*1000)}', round(largo * 1000), nota)] += 1
    return obj


def panel_red(nombre, origen, eje_u, eje_v, ancho, alto, col):
    o, u, v = Vector(origen), Vector(eje_u).normalized(), Vector(eje_v).normalized()
    verts = [o, o + u * ancho, o + u * ancho + v * alto, o + v * alto]
    me = bpy.data.meshes.new(nombre)
    me.from_pydata([tuple(p) for p in verts], [], [(0, 1, 2, 3)])
    uvl = me.uv_layers.new()
    for i, co in enumerate([(0, 0), (ancho, 0), (ancho, alto), (0, alto)]):
        uvl.data[i].uv = co
    me.materials.append(material_red())
    obj = bpy.data.objects.new(nombre, me)
    col.objects.link(obj)
    # relinga perimetral
    for i in range(4):
        cuerda(f'{nombre}_relinga{i}', verts[i], verts[(i + 1) % 4], col)
    return obj


def cuerda(nombre, a, b, col, r=0.003):
    a, b = Vector(a), Vector(b)
    me = bpy.data.meshes.get('cuerda_unit')
    if me is None:
        bpy.ops.mesh.primitive_cylinder_add(vertices=8, radius=1, depth=1)
        tmp = bpy.context.active_object
        me = tmp.data
        me.name = 'cuerda_unit'
        me.transform(Matrix.Translation((0, 0, 0.5)))  # de z=0 a z=1
        bpy.data.objects.remove(tmp, do_unlink=True)
        me.materials.append(material('relinga', (0.05, 0.05, 0.05), rugosidad=0.9))
    eje = b - a
    obj = bpy.data.objects.new(nombre, me)
    obj.matrix_world = (Matrix.Translation(a) @ eje.to_track_quat('Z', 'Y').to_matrix().to_4x4()
                        @ Matrix.Diagonal((r, r, eje.length, 1)))
    col.objects.link(obj)


def canamo(p_red, p_perfil, col):
    """Canamo M8 en tuerca-T: espiga desde la cara del perfil hasta el ojo en la red."""
    p_red, p_perfil = Vector(p_red), Vector(p_perfil)
    me = bpy.data.meshes.get('canamo_ojo')
    if me is None:
        bpy.ops.mesh.primitive_torus_add(major_radius=0.012, minor_radius=0.003,
                                         major_segments=12, minor_segments=6)
        tmp = bpy.context.active_object
        me = tmp.data
        me.name = 'canamo_ojo'
        bpy.data.objects.remove(tmp, do_unlink=True)
        me.materials.append(material('zincado', (0.75, 0.75, 0.72), metal=1, rugosidad=0.3))
    d = (p_red - p_perfil)
    ojo = bpy.data.objects.new('canamo', me)
    # el plano del ojo contiene la direccion de la espiga
    ojo.matrix_world = Matrix.Translation(p_red) @ d.to_track_quat('X', 'Z').to_matrix().to_4x4() \
        @ Matrix.Rotation(math.pi / 2, 4, 'X')
    col.objects.link(ojo)
    cuerda('canamo_espiga', p_perfil, p_red - d.normalized() * 0.012, col, r=0.004)
    herrajes['Canamo M8 + tuerca-T M8 ranura 8 + brida/mosqueton'] += 1


def a_lo_largo(a, b, paso):
    a, b = Vector(a), Vector(b)
    n = max(1, round((b - a).length / paso))
    return [a.lerp(b, i / n) for i in range(n + 1)]


def camara(nombre, pos, col):
    objetivo = Vector(((X0 + X1) / 2, (Y0 + Y1) / 2, 0.6))
    pos = Vector(pos)
    q = (objetivo - pos).to_track_quat('-Z', 'Y')
    mat_cuerpo = material('camara', (0.06, 0.06, 0.07), rugosidad=0.4)
    mat_lente = material('lente_ir', (0.5, 0.05, 0.05), metal=0.3, rugosidad=0.1)
    bpy.ops.mesh.primitive_cube_add(size=1)
    c = bpy.context.active_object
    c.name, c.scale = nombre, (0.126, 0.126, 0.13)
    c.data.materials.append(mat_cuerpo)
    bpy.ops.mesh.primitive_cylinder_add(radius=0.045, depth=0.02, location=(0, 0, -0.075))
    l = bpy.context.active_object
    l.name = nombre + '_anillo_ir'
    l.data.materials.append(mat_lente)
    l.parent = c
    l.scale = (1 / 0.126, 1 / 0.126, 1 / 0.13)
    l.location = (0, 0, -0.075 / 0.13)
    for o in (c, l):
        for cl in o.users_collection:
            cl.objects.unlink(o)
        col.objects.link(o)
    c.location = pos
    c.rotation_euler = q.to_euler()

    # soporte: placa en la cara exterior del anillo + brazo + rotula sobre la camara
    hacia_fuera = Vector((pos.x - objetivo.x, pos.y - objetivo.y, 0))
    hacia_fuera = Vector((1, 0, 0)) if abs(hacia_fuera.x) > abs(hacia_fuera.y) else Vector((0, 1, 0))
    hacia_fuera *= 1 if (pos - objetivo).dot(hacia_fuera) > 0 else -1
    cara = Vector((pos.x, pos.y, Z_ANILLO + H_ANILLO / 2))   # centro de la cara exterior
    if hacia_fuera.x:
        cara.x = X1 if hacia_fuera.x > 0 else X0
    else:
        cara.y = Y1 if hacia_fuera.y > 0 else Y0
    mat_sop = material('soporte_negro', (0.03, 0.03, 0.03), metal=0.5, rugosidad=0.5)
    bpy.ops.mesh.primitive_cube_add(size=1)
    placa = bpy.context.active_object
    placa.name = nombre + '_placa'
    placa.scale = (0.006 if hacia_fuera.x else 0.08, 0.006 if hacia_fuera.y else 0.08, 0.07)
    placa.location = cara + hacia_fuera * 0.003
    bpy.ops.mesh.primitive_uv_sphere_add(radius=0.022, segments=16, ring_count=8)
    rotula = bpy.context.active_object
    rotula.name = nombre + '_rotula'
    rotula.location = pos + Vector((0, 0, 0.09))
    for o in (placa, rotula):
        o.data.materials.append(mat_sop)
        for cl in o.users_collection:
            cl.objects.unlink(o)
        col.objects.link(o)
    codo = Vector((rotula.location.x, rotula.location.y, cara.z))
    cuerda(nombre + '_brazo', cara, codo, col, r=0.008)
    cuerda(nombre + '_mastil', codo, rotula.location, col, r=0.008)
    herrajes['Camara de tracking (p.ej. OptiTrack PrimeX 13 / Flex 13)'] += 1
    herrajes['Placa de montaje ranura 8 con tornillo 1/4"-20 + rotula 3 ejes'] += 1


# --- Construccion ---------------------------------------------------------------
def construir():
    vieja = bpy.data.collections.get('arena')
    if vieja:
        for o in list(vieja.all_objects):
            bpy.data.objects.remove(o, do_unlink=True)
        for c in list(vieja.children_recursive):
            bpy.data.collections.remove(c)
        bpy.data.collections.remove(vieja)
    for o in bpy.data.objects:
        if o.name.startswith('jaula_'):
            o.hide_set(True)
            o.hide_render = True
    corte.clear()
    herrajes.clear()

    raiz = coleccion('arena')
    c_est, c_red, c_her, c_cam = (coleccion(n, raiz) for n in
                                  ('estructura', 'red', 'herrajes', 'camaras'))
    alu = material('aluminio_anodizado', (0.80, 0.81, 0.83), metal=1, rugosidad=0.35)

    h = P / 2
    xm, ym = (X0 + X1) / 2, (Y0 + Y1) / 2
    xs, xn = X0 + h, X1 - h                 # ejes de los postes
    ys, yn = Y0 + h, Y1 - h
    yp0, yp1 = PUERTA_Y[0] - h, PUERTA_Y[1] + h
    postes = [(xs, ys), (xn, ys), (xs, yn), (xn, yn),          # esquinas
              (xm, ys), (xm, yn), (xs, ym),                      # media luz
              (xn, yp0), (xn, yp1)]                              # vano de puerta
    for i, (x, y) in enumerate(postes):
        perfil(f'poste{i}', (x, y, 0), (x, y, Z_ANILLO), P, P, c_est, alu,
               nota='poste, 2 caras con rosca M8 en testa')
    herrajes['Pie nivelador M8 + placa base para anclar a piso'] += len(postes)

    # anillo superior 40x80: largos corridos en sur/norte, cortos entre ellos
    zc = Z_ANILLO + H_ANILLO / 2
    for nombre, y in (('anillo_s', ys), ('anillo_n', yn)):
        perfil(nombre, (X0, y, zc), (X1, y, zc), H_ANILLO, P, c_est, alu,
               rodar=math.pi / 2, nota='anillo sup. corrido')
    for nombre, x in (('anillo_o', xs), ('anillo_e', xn)):
        perfil(nombre, (x, Y0 + P, zc), (x, Y1 - P, zc), H_ANILLO, P, c_est, alu,
               rodar=0, nota='anillo sup. entre largos')
    herrajes['Union de testa (conector automatico/ tornillo M8 en testa)'] += 4 + len(postes)

    # rieles inferiores entre postes (sin cruzar la puerta)
    def rieles(eje, fijo, cortes, prefijo):
        for i, (a, b) in enumerate(zip(cortes[:-1], cortes[1:])):
            if eje == 'x':
                perfil(f'{prefijo}{i}', (a + h, fijo, h), (b - h, fijo, h), P, P, c_est, alu,
                       rodar=math.pi / 2, nota='riel inferior')
            else:
                perfil(f'{prefijo}{i}', (fijo, a + h, h), (fijo, b - h, h), P, P, c_est, alu,
                       nota='riel inferior')
            herrajes['Escuadra 40 con tuercas-T (union riel-poste)'] += 2
    rieles('x', ys, [xs, xm, xn], 'riel_s')
    rieles('x', yn, [xs, xm, xn], 'riel_n')
    rieles('y', xs, [ys, ym, yn], 'riel_o')
    rieles('y', xn, [ys, yp0], 'riel_e_a')
    rieles('y', xn, [yp1, yn], 'riel_e_b')

    # cartelas a 45 grados en esquinas superiores, en el plano de cada pared
    k = 0
    for (x, y) in postes[:4]:
        dx = 1 if x < xm else -1
        dy = 1 if y < ym else -1
        for d in (Vector((dx, 0, 0)), Vector((0, dy, 0))):
            a = Vector((x, y, Z_ANILLO - CARTELA)) + d * h
            b = Vector((x, y, Z_ANILLO)) + d * CARTELA
            perfil(f'cartela{k}', a, b, P, P, c_est, alu,
                   rodar=0 if d.x == 0 else math.pi / 2, nota='cartela, cortes a 45')
            k += 1
    herrajes['Escuadra 40 con tuercas-T (union cartela)'] += 2 * k

    # --- red: plano a RED_OFFSET por dentro de la cara interior del perfil ---
    rx0, rx1 = X0 + P + RED_OFFSET, X1 - P - RED_OFFSET
    ry0, ry1 = Y0 + P + RED_OFFSET, Y1 - P - RED_OFFSET
    rz1 = Z_ANILLO - RED_OFFSET
    paneles = [
        ('red_sur', (rx0, ry0, 0), (1, 0, 0), rx1 - rx0, (0, -1, 0)),
        ('red_norte', (rx0, ry1, 0), (1, 0, 0), rx1 - rx0, (0, 1, 0)),
        ('red_oeste', (rx0, ry0, 0), (0, 1, 0), ry1 - ry0, (-1, 0, 0)),
        ('red_este_a', (rx1, ry0, 0), (0, 1, 0), PUERTA_Y[0] - ry0, (1, 0, 0)),
        ('red_este_b', (rx1, PUERTA_Y[1], 0), (0, 1, 0), ry1 - PUERTA_Y[1], (1, 0, 0)),
    ]
    area = 0.0
    for nombre, o, u, ancho, hacia_perfil in paneles:
        panel_red(nombre, o, u, (0, 0, 1), ancho, rz1, c_red)
        area += ancho * rz1
        o, u, n = Vector(o), Vector(u), Vector(hacia_perfil)
        borde = (a_lo_largo(o + Vector((0, 0, rz1)), o + u * ancho + Vector((0, 0, rz1)), RED_PASO)
                 + a_lo_largo(o + Vector((0, 0, 0.02)), o + u * ancho + Vector((0, 0, 0.02)), RED_PASO)[1:-1]
                 + a_lo_largo(o, o + Vector((0, 0, rz1)), RED_PASO)[1:-1]
                 + a_lo_largo(o + u * ancho, o + u * ancho + Vector((0, 0, rz1)), RED_PASO)[1:-1])
        for p in borde:
            canamo(p, p + n * RED_OFFSET, c_her)
    # techo de red colgado de la cara inferior del anillo
    o = Vector((rx0, ry0, rz1))
    panel_red('red_techo', o, (1, 0, 0), (0, 1, 0), rx1 - rx0, ry1 - ry0, c_red)
    area += (rx1 - rx0) * (ry1 - ry0)
    esquinas_t = [o, Vector((rx1, ry0, rz1)), Vector((rx1, ry1, rz1)), Vector((rx0, ry1, rz1))]
    for i in range(4):
        for p in a_lo_largo(esquinas_t[i], esquinas_t[(i + 1) % 4], RED_PASO)[1:]:
            canamo(p, p + Vector((0, 0, RED_OFFSET)), c_her)

    # cortina de puerta: panel suelto que traslapa el vano, colgado del anillo
    ancho_c = (PUERTA_Y[1] - PUERTA_Y[0]) + 2 * TRASLAPE_PUERTA
    xc = rx1 - 0.03
    panel_red('red_puerta_cortina', (xc, PUERTA_Y[0] - TRASLAPE_PUERTA, 0.0), (0, 1, 0), (0, 0, 1),
              ancho_c, rz1, c_red)
    area += ancho_c * rz1
    herrajes['Cierre de cremallera / velcro para traslapes de la cortina (m)'] += round(2 * rz1, 1)

    # camaras: colgadas de la cara EXTERIOR del anillo (fuera de la red), corridas
    # de esquinas, postes y cartelas para que nada tape la linea de vista
    zc_cam = Z_ANILLO - 0.16
    sal = 0.11                                 # centro de la camara fuera de la huella
    e = CARTELA + 0.10
    pos_cam = [(X0 + e, Y0 - sal), (xm + 0.30, Y0 - sal), (X1 - e, Y0 - sal),
               (X0 + e, Y1 + sal), (xm - 0.30, Y1 + sal), (X1 - e, Y1 + sal),
               (X0 - sal, ym + 0.30), (X1 + sal, (Y0 + PUERTA_Y[0]) / 2)]
    for i, (x, y) in enumerate(pos_cam[:N_CAMARAS]):
        camara(f'camara{i}', (x, y, zc_cam), c_cam)

    herrajes['Red sin nudos PE/nylon negra, malla 25 mm, hilo ~2 mm (m2)'] += round(area, 1)
    herrajes['Relinga (cuerda de borde) PP 6 mm cosida a la red (m)'] += round(
        2 * sum(ancho + rz1 for _, _, _, ancho, _ in paneles)
        + 2 * ((rx1 - rx0) + (ry1 - ry0)) + 2 * (ancho_c + rz1), 1)
    reporte()


def reporte():
    print('\n=== LISTA DE CORTE (perfil ranura 8, serie 40) ===')
    total = Counter()
    for (sec, largo, nota), n in sorted(corte.items()):
        print(f'{n:3d} x {sec:>5}  L = {largo:5d} mm   {nota}')
        total[sec] += n * largo
    for sec, mm in total.items():
        print(f'   total {sec}: {mm/1000:.2f} m')
    print('\n=== HERRAJES Y RED ===')
    for k, v in sorted(herrajes.items()):
        print(f'{v:>7} {k}')


if __name__ == '__main__':
    construir()
