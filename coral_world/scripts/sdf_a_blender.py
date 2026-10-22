#!/usr/bin/env python3
"""Importa worlds/coral.sdf en Blender (cajas y planos con sus colores).

Cada <visual> se convierte en un objeto, agrupado en colecciones por tipo
(muro, mesa, silla, ...). El piso de baldosas lleva su textura.

Uso:
  - Desde terminal:  blender --python scripts/sdf_a_blender.py
                     blender -b --python scripts/sdf_a_blender.py -- salida.blend
  - Desde Blender:   pestaña Scripting > Abrir este archivo > Run Script
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

import bpy

PAQUETE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(PAQUETE, 'worlds', 'coral.sdf')


def numeros(texto, n):
    vals = [float(v) for v in (texto or '').split()]
    return (vals + [0.0] * n)[:n]


def pose_de(elem):
    p = elem.find('pose')
    return numeros(p.text if p is not None else '', 6)


def sumar_pose(a, b):
    # Las poses de modelo/link del mundo CoRAL no tienen rotacion; basta sumar.
    return [x + y for x, y in zip(a, b)]


def material(color, textura=None):
    nombre = 'tex_' + os.path.basename(textura) if textura else \
        'mat_%.2f_%.2f_%.2f' % tuple(color[:3])
    mat = bpy.data.materials.get(nombre)
    if mat:
        return mat
    mat = bpy.data.materials.new(nombre)
    mat.use_nodes = True
    mat.diffuse_color = (*color[:3], 1.0)
    bsdf = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Roughness'].default_value = 0.7
    if textura and os.path.exists(textura):
        img = mat.node_tree.nodes.new('ShaderNodeTexImage')
        img.image = bpy.data.images.load(textura, check_existing=True)
        mat.node_tree.links.new(img.outputs['Color'], bsdf.inputs['Base Color'])
    else:
        bsdf.inputs['Base Color'].default_value = (*color[:3], 1.0)
    return mat


def malla_caja(sx, sy, sz):
    x, y, z = sx / 2, sy / 2, sz / 2
    verts = [(-x, -y, -z), (x, -y, -z), (x, y, -z), (-x, y, -z),
             (-x, -y, z), (x, -y, z), (x, y, z), (-x, y, z)]
    caras = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    me = bpy.data.meshes.new('caja')
    me.from_pydata(verts, [], caras)
    return me


def malla_plano(sx, sy):
    x, y = sx / 2, sy / 2
    me = bpy.data.meshes.new('plano')
    me.from_pydata([(-x, -y, 0), (x, -y, 0), (x, y, 0), (-x, y, 0)], [], [(0, 1, 2, 3)])
    uv = me.uv_layers.new()
    for i, co in enumerate([(0, 0), (1, 0), (1, 1), (0, 1)]):
        uv.data[i].uv = co
    return me


def coleccion(nombre, padre):
    col = bpy.data.collections.get(nombre)
    if col is None:
        col = bpy.data.collections.new(nombre)
        padre.children.link(col)
    return col


def importar(ruta=SDF):
    raiz = coleccion('coral', bpy.context.scene.collection)
    mundo = ET.parse(ruta).getroot().find('world')
    total = 0
    for modelo in mundo.findall('model'):
        pm = pose_de(modelo)
        for link in modelo.findall('link'):
            pl = sumar_pose(pm, pose_de(link))
            for vis in link.findall('visual'):
                geo = vis.find('geometry')
                caja, plano = geo.find('box'), geo.find('plane')
                if caja is not None:
                    me = malla_caja(*numeros(caja.findtext('size'), 3))
                elif plano is not None:
                    me = malla_plano(*numeros(plano.findtext('size'), 2))
                else:
                    continue

                color, textura = [0.8, 0.8, 0.8], None
                mat = vis.find('material')
                if mat is not None:
                    color = numeros(mat.findtext('diffuse') or '0.8 0.8 0.8', 3)
                    albedo = mat.findtext('.//albedo_map')
                    if albedo:
                        textura = os.path.join(PAQUETE, albedo.replace('model://coral_world/', ''))
                me.materials.append(material(color, textura))

                nombre = vis.get('name').removesuffix('_v')
                if modelo.get('name') != 'laboratorio_coral':
                    nombre = modelo.get('name')
                grupo = re.sub(r'(_[a-z]?\d+[a-z]?)+$', '', nombre)

                obj = bpy.data.objects.new(nombre, me)
                pose = sumar_pose(pl, pose_de(vis))
                obj.location = pose[:3]
                obj.rotation_euler = pose[3:]  # rpy de SDF = Euler XYZ de Blender
                coleccion(grupo, raiz).objects.link(obj)
                total += 1
    print('coral: %d objetos importados desde %s' % (total, ruta))


if __name__ == '__main__':
    args = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []
    if args:
        # Generando el .blend: partir de una escena vacia (sin el cubo por defecto)
        bpy.ops.wm.read_factory_settings(use_empty=True)
    importar()
    if args:
        bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(args[0]))
