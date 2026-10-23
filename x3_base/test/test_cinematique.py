# SPDX-License-Identifier: Apache-2.0
"""Odométrie et vitesse des roues mécanum de base_node (colcon test, ou python3 -m pytest)."""
import math

from x3_base.base_node import LX, LY, RAYON, integrer, vitesses_roues


def test_odometrie():
    # 1 s tout droit à 0,2 m/s, quart de tour, puis 1 s « tout droit » (= +y du repère odom)
    x, y, cap = integrer(0, 0, 0, 0.2, 0, 0, 1.0)
    assert (round(x, 6), y, cap) == (0.2, 0, 0)
    x, y, cap = integrer(x, y, cap, 0, 0, math.pi / 2, 1.0)
    x, y, cap = integrer(x, y, cap, 0.2, 0, 0, 1.0)
    assert abs(x - 0.2) < 1e-9 and abs(y - 0.2) < 1e-9
    assert integrer(0, 0, 0, 0, 0.1, 0, 1.0)[:2] == (0.0, 0.1)   # crabe vers la gauche : +y


def test_roues_mecanum():
    assert all(abs(w - 0.1 / RAYON) < 1e-9 for w in vitesses_roues(0.1, 0, 0))   # tout droit : 4 roues pareil
    gauche = vitesses_roues(0, 0.1, 0)                                             # crabe : diagonales opposées
    assert gauche[0] < 0 < gauche[1] and gauche[2] > 0 > gauche[3]
    rot = vitesses_roues(0, 0, 1.0)                                                # rotation à gauche : côté gauche recule
    assert rot[0] < 0 and rot[2] < 0 and rot[1] > 0 and rot[3] > 0
    assert abs(rot[1] - (LX + LY) / RAYON) < 1e-9
