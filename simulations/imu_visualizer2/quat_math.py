"""Quaternion math for the IMU visualizer.

Conventions
-----------
* Quaternions are (w, x, y, z), unit length, and represent the rotation of
  the IMU *body* frame relative to the *NED* world frame
  (North-East-Down): v_world = R(q) @ v_body.
* NED axes: X_n = North (forward), Y_n = East (right), Z_n = Down.
* OpenGL/camera frame: +X right, +Y up, -Z into the screen.

NED -> GL mapping (camera placed "south" of the body, looking north):
    E  -> +X_gl   (east is screen-right)
   -D  -> +Y_gl   (up is screen-up)
   -N  -> +Z_gl   (so -N points out of the screen, i.e. +N points into it)

A NED vector (n, e, d) therefore maps to GL vector (e, -d, -n), which is a
proper rotation (det = +1), so handedness is preserved.
"""

from __future__ import annotations

import ctypes
import math

import numpy as np

# Rotation matrix that maps NED coordinates to OpenGL camera coordinates.
# Rows are the NED components of each GL axis: x_gl = e, y_gl = -d, z_gl = -n.
_NED_TO_GL = np.array(
    [
        [0.0, 1.0, 0.0],  # E  -> +X_gl
        [0.0, 0.0, -1.0],  # -D -> +Y_gl
        [-1.0, 0.0, 0.0],  # -N -> +Z_gl
    ],
    dtype=np.float64,
)


def quat_normalize(qw: float, qx: float, qy: float, qz: float):
    """Return a unit quaternion. Falls back to identity for ~zero length."""
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (qw / n, qx / n, qy / n, qz / n)


def quat_sign_fix(q, q_ref):
    """Flip the sign of q if it lies in the opposite hemisphere from q_ref.

    q and -q encode the same rotation; for interpolation we always want the
    representative that is sign-continuous with the previous sample.
    """
    if sum(a * b for a, b in zip(q, q_ref)) < 0.0:
        return tuple(-v for v in q)
    return tuple(q)


def slerp(q0, q1, t: float):
    """Spherical linear interpolation between unit quaternions q0 and q1.

    Returns a normalized, sign-continuous quaternion (q1 is flipped first if
    dot(q0, q1) < 0 so the short path is taken). t is clamped to [0, 1].
    """
    q0 = quat_normalize(*q0)
    q1 = quat_sign_fix(quat_normalize(*q1), q0)

    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)

    dot = sum(a * b for a, b in zip(q0, q1))
    # Nearly identical orientations: fall back to nlerp to avoid sin(0) issues.
    if dot > 0.9995:
        q = [a + t * (b - a) for a, b in zip(q0, q1)]
        return quat_normalize(*q)

    dot = max(-1.0, min(1.0, dot))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    w0 = math.sin((1.0 - t) * theta) / sin_theta
    w1 = math.sin(t * theta) / sin_theta
    q = [w0 * a + w1 * b for a, b in zip(q0, q1)]
    return quat_normalize(*q)


def quat_to_mat3(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Convert a (w, x, y, z) quaternion to a 3x3 rotation matrix.

    The matrix maps body-frame vectors into the NED world frame:
    v_ned = R @ v_body.
    """
    qw, qx, qy, qz = quat_normalize(qw, qx, qy, qz)
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def ned_to_gl(m3: np.ndarray):
    """Convert a 3x3 body->NED rotation into a column-major 4x4 GL matrix.

    The returned matrix, when applied via glMultMatrixf, rotates the body
    model into the OpenGL camera frame: R_gl = M @ R_ned @ M^T, where
    M = _NED_TO_GL. Embedding in 4x4 with zero translation.
    """
    r_gl = _NED_TO_GL @ np.asarray(m3, dtype=np.float64) @ _NED_TO_GL.T
    m4 = np.identity(4, dtype=np.float64)
    m4[:3, :3] = r_gl
    # OpenGL expects column-major storage (Fortran order).
    flat = np.asfortranarray(m4).ravel(order="F")
    return (ctypes.c_float * 16)(*flat)
