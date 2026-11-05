#!/usr/bin/env python3
"""Draws the walls of a workshop world from the collision boxes of its
'estructura' model, so a coverage route can be planned offline.

    python3 tools/plot_world.py office_0 office_4
"""
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(HERE, '..', '..', 'worlds')
OUT = os.path.join(HERE, '..', '..', '..', 'docs', 'img')


def model_pose(world_sdf, prefix):
    """Pose of the <include> of the structure model, from the world SDF."""
    root = ET.parse(world_sdf).getroot()
    for inc in root.iter('include'):
        uri = inc.findtext('uri', '')
        if prefix in uri:
            pose = inc.findtext('pose', '0 0 0 0 0 0')
            return [float(v) for v in pose.split()]
    return [0.0] * 6


def boxes(model_sdf, pose):
    root = ET.parse(model_sdf).getroot()
    out = []
    for c in root.iter('collision'):
        size = c.findtext('geometry/box/size')
        if not size:
            continue
        p = c.findtext('pose', '0 0 0 0 0 0').split()
        x, y = float(p[0]) + pose[0], float(p[1]) + pose[1]
        sx, sy, sz = (float(v) for v in size.split())
        if sz < 1.0:          # floors and low furniture are not walls
            continue
        out.append((x - sx / 2, y - sy / 2, sx, sy))
    return out


def main():
    names = sys.argv[1:] or ['office_0', 'office_4']
    fig, axes = plt.subplots(1, len(names), figsize=(8 * len(names), 7))
    if len(names) == 1:
        axes = [axes]
    for ax, name in zip(axes, names):
        wsdf = os.path.join(WORLDS, 'worlds', f'{name}.sdf')
        msdf = os.path.join(WORLDS, 'models', f'oficina_{name.split("_")[1]}_estructura', 'model.sdf')
        pose = model_pose(wsdf, f'oficina_{name.split("_")[1]}_estructura')
        walls = boxes(msdf, pose)
        for x, y, sx, sy in walls:
            ax.add_patch(Rectangle((x, y), sx, sy, color='0.25'))
        ax.set_aspect('equal')
        ax.autoscale_view()
        ax.set_xlim(min(w[0] for w in walls) - 1, max(w[0] + w[2] for w in walls) + 1)
        ax.set_ylim(min(w[1] for w in walls) - 1, max(w[1] + w[3] for w in walls) + 1)
        ax.set_title(f'{name}: {len(walls)} wall boxes')
        ax.grid(alpha=0.3)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        print(f'{name}: pose {pose[:3]}, {len(walls)} walls')
    fig.tight_layout()
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, 'worlds_plan.png')
    fig.savefig(path, dpi=110)
    print(f'saved {os.path.normpath(path)}')


if __name__ == '__main__':
    main()
