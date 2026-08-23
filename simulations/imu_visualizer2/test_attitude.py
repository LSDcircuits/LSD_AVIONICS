#!/usr/bin/env python3
"""Unit tests for attitude_indicator.py — GL-free (no pyglet needed).

Checks quat_to_rpy_deg against the firmware's quat_to_euler() formulas and
the pure display-geometry helpers (ball_to_screen, bank_scale_pos).
Run:  python test_attitude.py
"""

import math
import unittest

from attitude_indicator import (
    quat_to_rpy_deg,
    ball_to_screen,
    bank_scale_pos,
)


def firmware_euler_deg(q):
    """Reference: exact formulas from the Pico firmware's quat_to_euler()."""
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(2.0 * (w * y - z * x))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


def axis_angle_quat(ax, ay, az, deg):
    """Unit quaternion for `deg` about unit axis (ax, ay, az)."""
    h = math.radians(deg) / 2.0
    s = math.sin(h)
    return (math.cos(h), ax * s, ay * s, az * s)


class TestQuatToRpy(unittest.TestCase):
    def test_identity(self):
        r, p, y = quat_to_rpy_deg((1.0, 0.0, 0.0, 0.0))
        self.assertAlmostEqual(r, 0.0)
        self.assertAlmostEqual(p, 0.0)
        self.assertAlmostEqual(y, 0.0)

    def test_matches_firmware_on_sample(self):
        # Real captured line: Q: 0.962 0.035 -0.043 -0.266 -> RPY 5.2 -3.7 -31.1
        q = (0.962, 0.035, -0.043, -0.266)
        r, p, y = quat_to_rpy_deg(q)
        self.assertAlmostEqual(r, 5.2, delta=0.3)
        self.assertAlmostEqual(p, -3.7, delta=0.3)
        self.assertAlmostEqual(y, -31.1, delta=0.3)

    def test_matches_firmware_formula_exactly(self):
        # quat_to_rpy_deg must agree with the firmware formula to float noise.
        quats = [
            (0.962, 0.035, -0.043, -0.266),
            axis_angle_quat(1, 0, 0, 45.0),
            axis_angle_quat(0, 1, 0, -30.0),
            axis_angle_quat(0, 0, 1, 120.0),
            axis_angle_quat(0.5, 0.5, 0.7071, 80.0),
            axis_angle_quat(-0.3, 0.8, -0.5196, -100.0),
        ]
        for q in quats:
            got = quat_to_rpy_deg(q)
            ref = firmware_euler_deg(q)
            for g, r_ in zip(got, ref):
                self.assertAlmostEqual(g, r_, places=5, msg=f"q={q}")

    def test_pure_rotations(self):
        r, p, y = quat_to_rpy_deg(axis_angle_quat(1, 0, 0, 90.0))
        self.assertAlmostEqual(r, 90.0, places=5)
        self.assertAlmostEqual(p, 0.0, places=5)
        self.assertAlmostEqual(y, 0.0, places=5)

        r, p, y = quat_to_rpy_deg(axis_angle_quat(0, 1, 0, 30.0))
        self.assertAlmostEqual(r, 0.0, places=5)
        self.assertAlmostEqual(p, 30.0, places=5)   # nose up = positive
        self.assertAlmostEqual(y, 0.0, places=5)

        r, p, y = quat_to_rpy_deg(axis_angle_quat(0, 0, 1, 45.0))
        self.assertAlmostEqual(r, 0.0, places=5)
        self.assertAlmostEqual(p, 0.0, places=5)
        self.assertAlmostEqual(y, 45.0, places=5)

    def test_non_unit_input_tolerated(self):
        # Printed quats (3 decimals) are slightly non-unit; must not crash
        # and must land within rounding error of the normalized result.
        q = (0.962, 0.035, -0.043, -0.266)
        a = quat_to_rpy_deg(q)
        b = quat_to_rpy_deg(tuple(v * 1.001 for v in q))
        for x, y_ in zip(a, b):
            self.assertAlmostEqual(x, y_, places=3)


class TestBallGeometry(unittest.TestCase):
    CX, CY, R = 320.0, 320.0, 128.0

    def test_horizon_tracks_pitch(self):
        # Pitch up 10 deg: the +10 ladder bar must land on the centre line
        # (aircraft symbol) — classic AI behaviour.
        ppd = self.R / 45.0
        x, y = ball_to_screen(self.CX, self.CY, 0.0, 10.0, ppd, 0.0, 10.0 * ppd)
        self.assertAlmostEqual(x, self.CX, places=6)
        self.assertAlmostEqual(y, self.CY, places=6)

    def test_right_roll_tilts_horizon_right_up(self):
        # Right roll (roll > 0): gravity in the pilot's view points
        # lower-right, so the horizon slopes up to the right — the right
        # end of the horizon rises (ball rotates CCW, i.e. +roll).
        ppd = self.R / 45.0
        _, y_right = ball_to_screen(self.CX, self.CY, 30.0, 0.0, ppd,
                                    self.R, 0.0)
        _, y_left = ball_to_screen(self.CX, self.CY, 30.0, 0.0, ppd,
                                   -self.R, 0.0)
        self.assertGreater(y_right, self.CY)
        self.assertLess(y_left, self.CY)

    def test_identity_ball_is_level(self):
        ppd = self.R / 45.0
        x, y = ball_to_screen(self.CX, self.CY, 0.0, 0.0, ppd, 0.0, 0.0)
        self.assertAlmostEqual(x, self.CX)
        self.assertAlmostEqual(y, self.CY)

    def test_pitch_up_drops_ground_below_centre(self):
        # With pitch up, the horizon (ly=0) moves below screen centre.
        ppd = self.R / 45.0
        _, y = ball_to_screen(self.CX, self.CY, 0.0, 20.0, ppd, 0.0, 0.0)
        self.assertLess(y, self.CY)


class TestBankScale(unittest.TestCase):
    CX, CY, R = 320.0, 320.0, 140.0

    def test_zero_at_top(self):
        x, y = bank_scale_pos(self.CX, self.CY, self.R, 0.0)
        self.assertAlmostEqual(x, self.CX)
        self.assertAlmostEqual(y, self.CY + self.R)

    def test_positive_bank_goes_right(self):
        x, _ = bank_scale_pos(self.CX, self.CY, self.R, 30.0)
        self.assertGreater(x, self.CX)
        x, _ = bank_scale_pos(self.CX, self.CY, self.R, -30.0)
        self.assertLess(x, self.CX)

    def test_symmetry(self):
        xr, yr = bank_scale_pos(self.CX, self.CY, self.R, 45.0)
        xl, yl = bank_scale_pos(self.CX, self.CY, self.R, -45.0)
        self.assertAlmostEqual(yr, yl)
        self.assertAlmostEqual(xr - self.CX, -(xl - self.CX))


if __name__ == "__main__":
    unittest.main(verbosity=2)
