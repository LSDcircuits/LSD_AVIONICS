"""Unit tests for the IMU visualizer.

Covers acceptance criteria 1 and 3 from SPEC.md:
  1. parse_line() extracts values from the real captured firmware lines
     (within 1e-3) and returns None for diagnostic/garbage lines.
  3. quat_to_mat3() produces correct rotations for known quaternions
     (90 deg about each axis), and slerp()/ned_to_gl() behave sanely.

Run:  python test_parser.py
"""

import ctypes
import math
import unittest

import numpy as np

from serial_parser import ImuSample, parse_line
from quat_math import (
    quat_normalize,
    quat_sign_fix,
    quat_to_mat3,
    slerp,
    ned_to_gl,
)

# --- Real captured samples from the firmware (SPEC.md section 2) ---------
CAPTURED_1 = (
    "G: -0.20  -0.55   0.10 dps | A:0.034 0.034 1.031 g | "
    "Q: 0.962  0.035 -0.043 -0.266 | RPY:  5.2  -3.7 -31.1 deg | "
    "HostUS:36740365"
)
CAPTURED_2 = (
    "G: -0.13  -0.27   0.03 dps | A:0.028 0.036 1.026 g | "
    "Q: 0.963  0.036 -0.037 -0.266 | RPY:  5.1  -3.0 -31.0 deg | "
    "HostUS:37370893"
)

# --- Known rotation quaternions (w, x, y, z) -----------------------------
S = math.sqrt(0.5)  # sin(45deg) = cos(45deg)
Q_IDENT = (1.0, 0.0, 0.0, 0.0)
Q_90_X = (S, S, 0.0, 0.0)  # +90 deg about +X_n (North)
Q_90_Y = (S, 0.0, S, 0.0)  # +90 deg about +Y_n (East)
Q_90_Z = (S, 0.0, 0.0, S)  # +90 deg about +Z_n (Down)


class TestParser(unittest.TestCase):
    def assertSampleAlmostEqual(self, sample: ImuSample, expected: dict):
        self.assertIsNotNone(sample)
        for key, value in expected.items():
            actual = getattr(sample, key)
            if isinstance(value, int):
                self.assertEqual(actual, value, key)
            else:
                self.assertAlmostEqual(actual, value, delta=1e-3, msg=key)

    def test_captured_line_1(self):
        self.assertSampleAlmostEqual(
            parse_line(CAPTURED_1),
            dict(
                gx=-0.20, gy=-0.55, gz=0.10,
                ax=0.034, ay=0.034, az=1.031,
                qw=0.962, qx=0.035, qy=-0.043, qz=-0.266,
                roll=5.2, pitch=-3.7, yaw=-31.1,
                host_us=36740365,
            ),
        )

    def test_captured_line_2(self):
        self.assertSampleAlmostEqual(
            parse_line(CAPTURED_2),
            dict(
                gx=-0.13, gy=-0.27, gz=0.03,
                ax=0.028, ay=0.036, az=1.026,
                qw=0.963, qx=0.036, qy=-0.037, qz=-0.266,
                roll=5.1, pitch=-3.0, yaw=-31.0,
                host_us=37370893,
            ),
        )

    def test_trailing_whitespace_and_crlf(self):
        s = parse_line(CAPTURED_1 + "  \r\n")
        self.assertIsNotNone(s)
        self.assertEqual(s.host_us, 36740365)

    def test_diagnostic_lines_ignored(self):
        self.assertIsNone(parse_line("WHO_AM_I=0x69"))
        self.assertIsNone(parse_line("CAL: gyro bias done"))
        self.assertIsNone(parse_line("CAL: accel bias done"))
        self.assertIsNone(parse_line("Gyro offsets: -0.20 -0.55 0.10"))
        self.assertIsNone(parse_line("LSM6DS3 init OK"))
        self.assertIsNone(parse_line("Mahony filter started"))

    def test_garbage_and_partials_ignored(self):
        self.assertIsNone(parse_line(""))
        self.assertIsNone(parse_line("\n"))
        self.assertIsNone(parse_line("hello world"))
        self.assertIsNone(parse_line("G: 1.0 2.0"))  # truncated line
        self.assertIsNone(parse_line("G: 1.0 2.0 3.0 dps | A:0.0 0.0 1.0 g"))  # partial
        self.assertIsNone(parse_line("Q: 1.0 0.0 0.0 0.0"))  # missing G/A sections
        self.assertIsNone(parse_line("G: a b c dps | A:0.0 0.0 1.0 g | "
                                     "Q:1 0 0 0 | RPY:0 0 0 deg | HostUS:1"))

    def test_sim_format_line_parses(self):
        # Line in the exact firmware printf format as produced by sim_serial.
        from sim_serial import SimSerial

        sim = SimSerial()
        line = sim.readline().decode()
        s = parse_line(line)
        self.assertIsNotNone(s)
        # Quaternion from the sim must be (close to) unit length.
        n = math.sqrt(s.qw**2 + s.qx**2 + s.qy**2 + s.qz**2)
        self.assertAlmostEqual(n, 1.0, delta=2e-3)


class TestQuatMath(unittest.TestCase):
    def assertMatAlmostEqual(self, R, expected, delta=1e-6):
        self.assertEqual(R.shape, (3, 3))
        np.testing.assert_allclose(R, np.array(expected, dtype=float), atol=delta)

    def test_identity(self):
        self.assertMatAlmostEqual(
            quat_to_mat3(*Q_IDENT),
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        )

    def test_90deg_about_x(self):
        # R(q) maps body->NED. +90deg about X_n: y_n -> z_n, z_n -> -y_n.
        self.assertMatAlmostEqual(
            quat_to_mat3(*Q_90_X),
            [[1, 0, 0], [0, 0, -1], [0, 1, 0]],
        )

    def test_90deg_about_y(self):
        # +90deg about Y_n: x_n -> -z_n, z_n -> x_n.
        self.assertMatAlmostEqual(
            quat_to_mat3(*Q_90_Y),
            [[0, 0, 1], [0, 1, 0], [-1, 0, 0]],
        )

    def test_90deg_about_z(self):
        # +90deg about Z_n: x_n -> y_n, y_n -> -x_n.
        self.assertMatAlmostEqual(
            quat_to_mat3(*Q_90_Z),
            [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
        )

    def test_rotation_preserves_vectors(self):
        # Columns of R are the body axes expressed in NED; check one directly:
        # +90deg about X_n maps body-Y onto world-Z (Down).
        R = quat_to_mat3(*Q_90_X)
        np.testing.assert_allclose(R @ np.array([0, 1, 0]), [0, 0, 1], atol=1e-9)

    def test_unnormalized_input(self):
        R1 = quat_to_mat3(*Q_90_Z)
        R2 = quat_to_mat3(2 * Q_90_Z[0], 2 * Q_90_Z[1], 2 * Q_90_Z[2], 2 * Q_90_Z[3])
        np.testing.assert_allclose(R1, R2, atol=1e-9)

    def test_zero_quaternion_is_identity(self):
        self.assertMatAlmostEqual(
            quat_to_mat3(0, 0, 0, 0),
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        )

    def test_orthonormal(self):
        R = quat_to_mat3(0.962, 0.035, -0.043, -0.266)
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=9)

    def test_slerp_endpoints(self):
        q0 = Q_IDENT
        q1 = Q_90_Z
        np.testing.assert_allclose(slerp(q0, q1, 0.0), q0, atol=1e-9)
        np.testing.assert_allclose(slerp(q0, q1, 1.0), q1, atol=1e-9)

    def test_slerp_halfway(self):
        # Halfway between identity and +90deg about Z is +45deg about Z.
        s45, c45 = math.sin(math.pi / 8), math.cos(math.pi / 8)
        q_mid = slerp(Q_IDENT, Q_90_Z, 0.5)
        np.testing.assert_allclose(q_mid, (c45, 0, 0, s45), atol=1e-9)

    def test_slerp_sign_continuity(self):
        # q1 = -Q_90_Z encodes the SAME rotation as Q_90_Z; slerp must take
        # the short path and end up at the physical orientation, and the
        # result must be sign-continuous with q0.
        q1_neg = (-Q_90_Z[0], -Q_90_Z[1], -Q_90_Z[2], -Q_90_Z[3])
        q_mid = slerp(Q_IDENT, q1_neg, 1.0)
        np.testing.assert_allclose(q_mid, Q_90_Z, atol=1e-9)
        # Midpoint must equal the short-path midpoint too.
        q_half = slerp(Q_IDENT, q1_neg, 0.5)
        np.testing.assert_allclose(q_half, slerp(Q_IDENT, Q_90_Z, 0.5), atol=1e-9)

    def test_slerp_normalized_output(self):
        q = slerp((0.96, 0.03, -0.04, -0.26), (0.5, 0.5, 0.5, 0.5), 0.37)
        self.assertAlmostEqual(sum(v * v for v in q), 1.0, places=9)

    def test_slerp_t_clamped(self):
        np.testing.assert_allclose(slerp(Q_IDENT, Q_90_Z, -0.5), Q_IDENT, atol=1e-9)
        np.testing.assert_allclose(slerp(Q_IDENT, Q_90_Z, 1.5), Q_90_Z, atol=1e-9)

    def test_sign_fix(self):
        q = (-0.5, -0.5, -0.5, -0.5)
        fixed = quat_sign_fix(q, Q_IDENT)
        self.assertEqual(fixed, (0.5, 0.5, 0.5, 0.5))
        self.assertEqual(quat_sign_fix(Q_IDENT, Q_IDENT), Q_IDENT)

    def test_ned_to_gl_type_and_shape(self):
        m = ned_to_gl(quat_to_mat3(*Q_IDENT))
        self.assertIsInstance(m, ctypes.Array)
        self.assertEqual(len(m), 16)
        self.assertTrue(all(isinstance(v, float) for v in m))

    def test_ned_to_gl_identity_rotation_is_identity(self):
        # ned_to_gl conjugates by the (orthogonal, det=+1) NED->GL change of
        # basis, so an identity body rotation must yield an identity GL
        # rotation with zero translation. (The axis convention E->+X_gl,
        # -D->+Y_gl, -N->+Z_gl is exercised by the rotation test below.)
        m = ned_to_gl(quat_to_mat3(*Q_IDENT))
        M = np.array(m, dtype=float).reshape(4, 4, order="F")  # back to math layout
        np.testing.assert_allclose(M, np.eye(4), atol=1e-6)

    def test_ned_to_gl_preserves_handedness(self):
        # The NED->GL basis change is a proper rotation, so det stays +1.
        m = ned_to_gl(quat_to_mat3(*Q_90_X))
        R = np.array(m, dtype=float).reshape(4, 4, order="F")[:3, :3]
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=6)

    def test_ned_to_gl_rotation_consistent(self):
        # NED Down maps to GL -Y, so +90deg about NED Down (per Rodrigues
        # about axis (0,-1,0)) must send GL +X to GL +Z.
        m = ned_to_gl(quat_to_mat3(*Q_90_Z))
        R = np.array(m, dtype=float).reshape(4, 4, order="F")[:3, :3]
        np.testing.assert_allclose(R @ [1, 0, 0], [0, 0, 1], atol=1e-6)
        # Round-trip sanity: conjugating back recovers the NED rotation.
        from quat_math import _NED_TO_GL

        R_ned = _NED_TO_GL.T @ R @ _NED_TO_GL
        np.testing.assert_allclose(R_ned, quat_to_mat3(*Q_90_Z), atol=1e-6)


class TestSimConsistency(unittest.TestCase):
    """Cross-check sim_serial's RPY against quat_to_mat3 (sanity only)."""

    def test_sim_rpy_matches_matrix(self):
        from sim_serial import quat_to_rpy_deg, quat_from_axis_angle

        # +90deg about Down => pure yaw = +90deg.
        roll, pitch, yaw = quat_to_rpy_deg(Q_90_Z)
        self.assertAlmostEqual(roll, 0.0, delta=1e-9)
        self.assertAlmostEqual(pitch, 0.0, delta=1e-9)
        self.assertAlmostEqual(yaw, 90.0, delta=1e-9)
        # +90deg about North => pure roll = +90deg.
        roll, pitch, yaw = quat_to_rpy_deg(Q_90_X)
        self.assertAlmostEqual(roll, 90.0, delta=1e-9)
        self.assertAlmostEqual(pitch, 0.0, delta=1e-9)
        self.assertAlmostEqual(yaw, 0.0, delta=1e-9)
        # quat_from_axis_angle agrees with the known constants.
        qz = quat_from_axis_angle((0, 0, 1), math.pi / 2)
        np.testing.assert_allclose(qz, Q_90_Z, atol=1e-12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
