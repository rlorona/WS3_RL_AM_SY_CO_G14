#!/usr/bin/env python3
"""Offline check of the figures in trajectory.py (no Gazebo, no ROS).

Samples every figure at the controller rate, draws it and reports:
  - the bounding box and the size,
  - the largest jump between two consecutive samples (a jump means the chain
    has a discontinuity),
  - the start/end pose,
  - for 'grupo14', the closing error.

Usage:
    python3 tools/validate_figures.py            # all figures
    python3 tools/validate_figures.py RASC       # only one
"""
import importlib.util
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
TRAJECTORY = os.path.join(HERE, '..', 'omni_workshop', 'trajectory.py')
OUT_DIR = os.path.join(HERE, '..', '..', '..', 'docs', 'img')
RATE = 20.0

spec = importlib.util.spec_from_file_location('trajectory', TRAJECTORY)
trajectory = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trajectory)


def sample(figure):
    n = max(2, int(math.ceil(figure.duration * RATE)) + 1)
    dt = figure.duration / (n - 1)
    return [figure(i * dt) for i in range(n)], dt


def check(name, figure):
    samples, dt = sample(figure)
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    max_step = 0.0
    for a, b in zip(samples, samples[1:]):
        max_step = max(max_step, math.hypot(b[0] - a[0], b[1] - a[1]))
    start = samples[0][:3]
    end = samples[-1][:3]
    close = math.hypot(end[0] - start[0], end[1] - start[1])

    print(f'\n{name}')
    print(f'  duration      : {figure.duration:6.2f} s')
    print(f'  bounding box  : x [{min(xs):+.3f}, {max(xs):+.3f}]  '
          f'y [{min(ys):+.3f}, {max(ys):+.3f}]')
    print(f'  size          : {max(xs) - min(xs):.3f} x {max(ys) - min(ys):.3f} m')
    print(f'  start -> end  : ({start[0]:+.3f}, {start[1]:+.3f}) -> '
          f'({end[0]:+.3f}, {end[1]:+.3f})')
    print(f'  closing error : {close * 100:.2f} cm')
    print(f'  largest step  : {max_step * 100:.2f} cm  '
          f'(speed {max_step / dt:.3f} m/s, dt {dt:.3f} s)')
    if max_step > 0.25:
        print('  WARNING: discontinuity in the chain')
    return samples


def plot(names):
    fig, axes = plt.subplots(1, len(names), figsize=(5 * len(names), 5))
    if len(names) == 1:
        axes = [axes]
    for ax, name in zip(axes, names):
        figure = trajectory.FIGURES[name]()
        samples, _ = sample(figure)
        ax.plot([s[0] for s in samples], [s[1] for s in samples], lw=2)
        ax.plot(samples[0][0], samples[0][1], 'go', ms=9, label='start')
        ax.plot(samples[-1][0], samples[-1][1], 'rs', ms=9, label='end')
        ax.set_aspect('equal', adjustable='box')
        ax.grid(alpha=0.3)
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title(name)
        ax.legend(fontsize=8)
    fig.tight_layout()
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, 'figures_check.png')
    fig.savefig(path, dpi=120)
    print(f'\nFigure saved to {os.path.normpath(path)}')


def main():
    names = sys.argv[1:] or list(trajectory.FIGURES)
    for name in names:
        if name not in trajectory.FIGURES:
            raise SystemExit(f'Unknown figure "{name}". Options: {", ".join(trajectory.FIGURES)}')
        check(name, trajectory.FIGURES[name]())
    plot(names)


if __name__ == '__main__':
    main()
