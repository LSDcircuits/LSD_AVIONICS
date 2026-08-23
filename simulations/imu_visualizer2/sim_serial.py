"""Synthetic IMU telemetry generator.

Used by --sim mode (no hardware attached). Generates a smooth tumbling
attitude by integrating a slowly-varying angular velocity at 100 Hz, and
can also emit lines in the exact firmware printf format (useful for testing
the serial path end-to-end).

The SimSerial class mimics the tiny subset of the pyserial API used by the
reader thread (readline / close), so the same reader code path runs
unchanged in simulation.
"""

from __future__ import annotations

import math
import time

from quat_math import quat_normalize

SIM_RATE_HZ = 100.0


def quat_from_axis_angle(axis, angle_rad: float):
    """Unit quaternion (w, x, y, z) for a rotation of angle about axis."""
    n = math.sqrt(sum(a * a for a in axis))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    s = math.sin(angle_rad / 2.0) / n
    return quat_normalize(math.cos(angle_rad / 2.0), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_mul(a, b):
    """Hamilton product of quaternions (w, x, y, z). a*b applies b then a."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_to_rpy_deg(q):
    """Roll/pitch/yaw (deg) from a body-relative-to-NED quaternion.

    Aerospace convention: roll about North (x_n), pitch about East (y_n),
    yaw about Down (z_n), applied in Z-Y-X (yaw-pitch-roll) order.
    """
    w, x, y, z = quat_normalize(*q)
    # roll (x_n axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y_n axis)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    # yaw (z_n axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class SimSerial:
    """Drop-in replacement for serial.Serial producing synthetic samples.

    Generates attitude at SIM_RATE_HZ by integrating a smooth, slowly
    meandering angular velocity, and derives plausible gyro/accel values.
    readline() blocks for the sample period and returns one formatted line
    in the exact firmware format.
    """

    def __init__(self, rate_hz: float = SIM_RATE_HZ):
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.is_open = True
        self._t0 = time.monotonic()
        self._host_us0 = 36_000_000  # plausible firmware uptime
        self._q = (1.0, 0.0, 0.0, 0.0)
        self._gyro = (0.0, 0.0, 0.0)

    # -- simulation step -------------------------------------------------
    def _step(self, t: float):
        # Smooth tumbling: angular velocity (rad/s, body frame) wanders on
        # three incommensurate frequencies so the motion never repeats.
        wx = 0.9 * math.sin(2 * math.pi * 0.10 * t)
        wy = 0.7 * math.sin(2 * math.pi * 0.13 * t + 1.0)
        wz = 0.8 * math.sin(2 * math.pi * 0.07 * t + 2.0)
        self._gyro = (math.degrees(wx), math.degrees(wy), math.degrees(wz))

        angle = math.sqrt(wx * wx + wy * wy + wz * wz) * self.dt
        if angle > 1e-12:
            dq = quat_from_axis_angle((wx, wy, wz), angle)
            self._q = quat_normalize(*quat_mul(self._q, dq))

        # Accel in the body frame = gravity vector (NED z_n = Down, i.e.
        # (0,0,1) in NED with +g along Down) rotated into the body frame.
        # v_body = R(q)^T @ (0, 0, 1).
        w, x, y, z = self._q
        # Third column of R^T = third row of R(q).
        ax = 2.0 * (x * z - w * y)
        ay = 2.0 * (y * z + w * x)
        az = 1.0 - 2.0 * (x * x + y * y)
        self._accel = (ax, ay, az)

    def readline(self) -> bytes:
        if not self.is_open:
            return b""
        time.sleep(self.dt)
        t = time.monotonic() - self._t0
        self._step(t)
        host_us = self._host_us0 + int(t * 1_000_000)
        gx, gy, gz = self._gyro
        ax, ay, az = self._accel
        roll, pitch, yaw = quat_to_rpy_deg(self._q)
        qw, qx, qy, qz = self._q
        line = (
            f"G:{gx:6.2f} {gy:6.2f} {gz:6.2f} dps | "
            f"A:{ax:5.3f} {ay:5.3f} {az:5.3f} g | "
            f"Q:{qw:6.3f} {qx:6.3f} {qy:6.3f} {qz:6.3f} | "
            f"RPY:{roll:5.1f} {pitch:5.1f} {yaw:5.1f} deg | "
            f"HostUS:{host_us}\n"
        )
        return line.encode("ascii")

    def close(self):
        self.is_open = False

    def open(self):
        self.is_open = True


if __name__ == "__main__":
    # Print a few seconds of synthetic telemetry for inspection.
    sim = SimSerial()
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
        print(sim.readline().decode(), end="")
