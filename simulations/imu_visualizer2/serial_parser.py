"""Pure regex parser for Pico IMU telemetry lines.

Firmware printf format (authoritative):

    G:%6.2f %6.2f %6.2f dps | A:%5.3f %5.3f %5.3f g | Q:%6.3f %6.3f %6.3f %6.3f | RPY:%5.1f %5.1f %5.1f deg | HostUS:%llu

Example captured line:

    G: -0.20  -0.55   0.10 dps | A:0.034 0.034 1.031 g | Q: 0.962  0.035 -0.043 -0.266 | RPY:  5.2  -3.7 -31.1 deg | HostUS:36740365

Quaternion order is w, x, y, z and expresses the body frame relative to a
NED (North-East-Down) world frame.

Boot / diagnostic lines (WHO_AM_I=..., CAL: ..., Gyro offsets: ...) and any
partial or malformed line are ignored: parse_line() returns None instead of
raising.
"""

from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass
class ImuSample:
    gx: float
    gy: float
    gz: float  # gyro, degrees/second
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
    host_us: int  # firmware-side microsecond timestamp


# Match signed decimals with flexible whitespace. Anchor on the 'G:' prefix
# so random diagnostic garbage cannot be mistaken for telemetry.
_FLOAT = r"(-?\d+(?:\.\d+)?)"

_LINE_RE = re.compile(
    r"^\s*G:\s*"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s*dps\s*\|\s*A:\s*"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s*g\s*\|\s*Q:\s*"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s*\|\s*RPY:\s*"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s+"
    + _FLOAT
    + r"\s*deg\s*\|\s*HostUS:\s*(\d+)"
)


def parse_line(line: str) -> ImuSample | None:
    """Parse one telemetry line into an ImuSample, or None if it does not
    match the firmware telemetry format (diagnostics, garbage, partials)."""
    if not line:
        return None
    m = _LINE_RE.match(line)
    if m is None:
        return None
    try:
        (
            gx,
            gy,
            gz,
            ax,
            ay,
            az,
            qw,
            qx,
            qy,
            qz,
            roll,
            pitch,
            yaw,
        ) = (float(g) for g in m.groups()[:13])
        host_us = int(m.group(14))
    except (ValueError, TypeError, IndexError):
        return None
    return ImuSample(
        gx=gx,
        gy=gy,
        gz=gz,
        ax=ax,
        ay=ay,
        az=az,
        qw=qw,
        qx=qx,
        qy=qy,
        qz=qz,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        host_us=host_us,
    )
