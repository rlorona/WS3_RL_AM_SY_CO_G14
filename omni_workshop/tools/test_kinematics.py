#!/usr/bin/env python3
"""Checks the mecanum model: IK -> FK must give back the same velocity.

Run with the workshop sourced (it imports the node module):

    source /opt/ros/jazzy/setup.bash
    source ~/workshop_03/install/setup.bash
    python3 tools/test_kinematics.py
"""
import math
import random
import sys

from omni_workshop.self_controller import (R, LX, LY, forward_kinematics,
                                          inverse_kinematics, saturate,
                                          world_to_robot)

random.seed(14)


def test_round_trip():
    worst = 0.0
    for _ in range(1000):
        vx = random.uniform(-1.0, 1.0)
        vy = random.uniform(-1.0, 1.0)
        w = random.uniform(-1.5, 1.5)
        got = forward_kinematics(*inverse_kinematics(vx, vy, w))
        worst = max(worst, max(abs(a - b) for a, b in zip(got, (vx, vy, w))))
    assert worst < 1e-9, worst
    print(f'IK -> FK round trip: max error {worst:.2e}  OK')


def test_world_to_robot():
    # Facing +y (90 deg), moving +x in the world is to the robot's right
    vx, vy, w = world_to_robot(math.pi / 2, 1.0, 0.0, 0.0)
    assert abs(vx) < 1e-12 and abs(vy + 1.0) < 1e-12 and w == 0.0, (vx, vy, w)
    # Facing +y, moving +y in the world is forward
    vx, vy, w = world_to_robot(math.pi / 2, 0.0, 1.0, 0.0)
    assert abs(vx - 1.0) < 1e-12 and abs(vy) < 1e-12, (vx, vy, w)
    # Facing +x, moving +y in the world must be +y in the robot frame
    vx, vy, w = world_to_robot(0.0, 0.0, 1.0, 0.3)
    assert abs(vx) < 1e-12 and abs(vy - 1.0) < 1e-12 and abs(w - 0.3) < 1e-12
    print('world -> robot frame: OK')


def test_known_cases():
    # Forward at 0.2 m/s: the four wheels turn at v/r
    wheels = inverse_kinematics(0.2, 0.0, 0.0)
    assert all(abs(v - 0.2 / R) < 1e-12 for v in wheels), wheels
    # Pure rotation: left wheels backwards, right wheels forwards
    fl, fr, rl, rr = inverse_kinematics(0.0, 0.0, 1.0)
    assert fl < 0 and rl < 0 and fr > 0 and rr > 0, (fl, fr, rl, rr)
    assert abs(fl + fr) < 1e-12 and abs(rl + rr) < 1e-12
    # Pure sideways (+y, to the left): diagonals opposite
    fl, fr, rl, rr = inverse_kinematics(0.0, 0.1, 0.0)
    assert abs(fl - rr) < 1e-12 and abs(fr - rl) < 1e-12
    print('known cases (forward, rotate, sideways): OK')


def test_saturation():
    wheels = (30.0, -5.0, 5.0, 5.0)
    out = saturate(wheels, limit=15.0)
    assert max(abs(v) for v in out) <= 15.0 + 1e-12
    # The ratios between the wheels must be preserved
    scale = 15.0 / 30.0
    assert all(abs(a - b * scale) < 1e-9 for a, b in zip(out, wheels)), (out, wheels)
    print('wheel saturation keeps the direction: OK')


if __name__ == '__main__':
    print(f'Dingo-O: r={R} m, lx={LX} m, ly={LY} m')
    test_round_trip()
    test_world_to_robot()
    test_known_cases()
    test_saturation()
    print('\nAll kinematic checks passed.')
    sys.exit(0)
