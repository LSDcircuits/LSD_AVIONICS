"""Parser for Pico IMU telemetry lines.

Supports both the original firmware format:

    G:%6.2f %6.2f %6.2f dps | A:%5.3f %5.3f %5.3f g | Q:%6.3f %6.3f %6.3f %6.3f | RPY:%5.1f %5.1f %5.1f deg | HostUS:%llu

and the newer format without HostUS and with gyro in rad/s:

    G: -0.00  -0.00  -0.05 rad | A:-0.007 0.073 1.000 g | Q:-0.995 -0.041 -0.007 -0.089 | RPY:  4.8   0.4  10.2 deg

Gyro is normalized internally to degrees/second so the rest of the
visualizer keeps working unchanged. If HostUS is absent it is set to 0,
which makes imu_visualizer.interpolated_attitude() fall back to arrival
wall-clock timing.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass


@dataclass
class ImuSample:
    gx: float
    gy: float
    gz: float  # gyro, degrees/second (normalized by the parser)
    ax: float
    ay: float
    az: float  # accel, g
    qw: float
    qx: float
    qy: float
    qz: float  # attitude quaternion (w, x, y, z), body-relative-to-NED
    roll: float
    pitch: float
    yaw: float  # degrees
    host_us: int  # firmware-side microsecond timestamp; 0 if not streamed


_FLOAT = r"(-?\d+(?:\.\d+)?)"

_LINE_RE = re.compile(
    r"^\s*G:\s*(?P<gx>" + _FLOAT + r")\s+(?P<gy>" + _FLOAT + r")\s+(?P<gz>" + _FLOAT +
    r")\s*(?P<gunit>dps|rad|rps)\s*\|\s*A:\s*(?P<ax>" + _FLOAT + r")\s+(?P<ay>" + _FLOAT +
    r")\s+(?P<az>" + _FLOAT + r")\s*g\s*\|\s*Q:\s*(?P<qw>" + _FLOAT + r")\s+(?P<qx>" + _FLOAT +
    r")\s+(?P<qy>" + _FLOAT + r")\s+(?P<qz>" + _FLOAT + r")\s*\|\s*RPY:\s*(?P<roll>" + _FLOAT +
    r")\s+(?P<pitch>" + _FLOAT + r")\s+(?P<yaw>" + _FLOAT + r")\s*deg"
    r"(?:\s*\|\s*HostUS:\s*(?P<host>\d+))?\s*$",
    re.IGNORECASE,
)

_GYRO_SCALE = {"dps": 1.0, "rad": 180.0 / math.pi, "rps": 360.0}


def parse_line(line: str) -> ImuSample | None:
    """Parse one telemetry line into an ImuSample, or None if it does not
    match the telemetry format (diagnostics, garbage, partials)."""
    if not line:
        return None
    m = _LINE_RE.match(line)
    if m is None:
        return None
    try:
        gd = m.groupdict()
        scale = _GYRO_SCALE[gd["gunit"].lower()]
        host = gd.get("host")
        return ImuSample(
            gx=float(gd["gx"]) * scale,
            gy=float(gd["gy"]) * scale,
            gz=float(gd["gz"]) * scale,
            ax=float(gd["ax"]),
            ay=float(gd["ay"]),
            az=float(gd["az"]),
            qw=float(gd["qw"]),
            qx=float(gd["qx"]),
            qy=float(gd["qy"]),
            qz=float(gd["qz"]),
            roll=float(gd["roll"]),
            pitch=float(gd["pitch"]),
            yaw=float(gd["yaw"]),
            host_us=int(host) if host else 0,
        )
    except (ValueError, TypeError, KeyError):
        return None
