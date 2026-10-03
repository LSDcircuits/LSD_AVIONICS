#!/usr/bin/env python3
"""Drop-in replacement for attitude_indicator.py with a rebuilt data layer.

Same instrument/renderer as the original, but self-contained: the serial
reader, parser, slerp and quaternion math are implemented HERE instead of
imported from imu_visualizer/serial_parser/quat_math. Built against the
observed firmware line format:

    G: <gx> <gy> <gz> rad | A:<ax> <ay> <az> g | Q: <w> <x> <y> <z> | RPY: ...

Lines without a "Q:" field (e.g. "serial workjing") are skipped; partial
lines from chunked reads are buffered until a newline arrives.

Usage (identical to the original):
    python3 attitude_indicator_fixed.py --port /dev/cu.usbmodem1101
    python3 attitude_indicator_fixed.py --sim
    python3 attitude_indicator_fixed.py --list
    python3 attitude_indicator_fixed.py --headless-test 5
"""

from __future__ import annotations

import argparse
import collections
import math
import re
import sys
import threading
import time

# --- third-party ------------------------------------------------------------
import serial
from serial.tools import list_ports as _list_ports


# ---------------------------------------------------------------------------
# Quaternion math (self-contained)
# ---------------------------------------------------------------------------

def quat_to_mat3(w, x, y, z):
    """Rotation matrix (body-to-NED) from a Hamilton quaternion (w,x,y,z).
    Normalizes first, so slightly off-norm telemetry is fine."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)),
        (2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)),
    )


def quat_to_rpy_deg(q):
    """(roll, pitch, yaw) degrees, ZYX/NED — same convention as the firmware."""
    R = quat_to_mat3(*q)
    roll = math.degrees(math.atan2(R[2][1], R[2][2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0]))))
    yaw = math.degrees(math.atan2(R[1][0], R[0][0]))
    return roll, pitch, yaw


def slerp(q0, q1, t):
    """Spherical interpolation between unit quats, shortest path."""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    dot = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1
    if dot < 0.0:                       # take the short way round
        w1, x1, y1, z1 = -w1, -x1, -y1, -z1
        dot = -dot
    if dot > 0.9995:                    # nearly parallel: lerp
        w = w0 + t * (w1 - w0)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
        n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
        return (w / n, x / n, y / n, z / n)
    dot = max(-1.0, min(1.0, dot))
    theta = math.acos(dot)
    s = math.sin(theta) or 1e-12
    a = math.sin((1.0 - t) * theta) / s
    b = math.sin(t * theta) / s
    return (a * w0 + b * w1, a * x0 + b * x1, a * y0 + b * y1, a * z0 + b * z1)


def quat_from_rpy_deg(roll, pitch, yaw=0.0):
    """Euler (deg, ZYX) -> quaternion. Used by the built-in simulator."""
    r, p, y = (math.radians(v) for v in (roll, pitch, yaw))
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


# ---------------------------------------------------------------------------
# Parser (self-contained) — matches the observed firmware output
# ---------------------------------------------------------------------------

_QPAT = re.compile(
    r"Q:\s*(-?\d+\.?\d*(?:[eE][-+]?\d+)?)\s+"
    r"(-?\d+\.?\d*(?:[eE][-+]?\d+)?)\s+"
    r"(-?\d+\.?\d*(?:[eE][-+]?\d+)?)\s+"
    r"(-?\d+\.?\d*(?:[eE][-+]?\d+)?)")


class ImuSample:
    __slots__ = ("t", "quat")

    def __init__(self, t, quat):
        self.t = t          # arrival time, time.monotonic()
        self.quat = quat    # (w, x, y, z)

    def __repr__(self):
        return f"ImuSample(t={self.t:.3f}, quat={self.quat})"


def parse_line(line):
    """ImuSample from one firmware line, or None if it carries no quaternion."""
    if isinstance(line, bytes):
        line = line.decode("ascii", "ignore")
    m = _QPAT.search(line)
    if not m:
        return None
    try:
        q = tuple(float(v) for v in m.groups())
    except ValueError:
        return None
    return ImuSample(time.monotonic(), q)


# ---------------------------------------------------------------------------
# TelemetryReader (self-contained)
# ---------------------------------------------------------------------------

class TelemetryReader:
    """Background serial reader.

    - Reads the port in chunks, buffers partial lines, parses complete ones.
    - Keeps a deque of (t, quat); reader.latest() returns that deque.
    - NEVER swallows exceptions silently: a read/parse failure is written to
      self.status so the HUD shows what actually happened.
    """

    def __init__(self, port=None, baud=115200, sim=False):
        self.port = port
        self.baud = baud
        self.sim = sim
        self.connected = False
        self.rate_hz = 0.0
        self.status = "idle"
        self._samples = collections.deque(maxlen=1024)
        self._ser = None
        self._thread = None
        self._running = False
        self._count = 0
        self._rate_t0 = time.monotonic()

    # -- lifecycle ---------------------------------------------------------
    def start(self):
        self._running = True
        if self.sim:
            self.connected = True
            self.status = "simulated telemetry"
        else:
            try:
                self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
            except Exception as exc:
                self.status = f"open failed: {type(exc).__name__}: {exc}"
                return
            self.connected = True
            self.status = f"connected {self._ser.name}"
        self._thread = threading.Thread(target=self._loop, name="telemetry",
                                        daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass

    # -- producer ----------------------------------------------------------
    def _push(self, quat):
        self._samples.append((time.monotonic(), quat))
        self._count += 1
        now = time.monotonic()
        dt = now - self._rate_t0
        if dt >= 1.0:
            self.rate_hz = self._count / dt
            self._count = 0
            self._rate_t0 = now

    def _loop(self):
        if self.sim:
            self._loop_sim()
        else:
            self._loop_serial()

    def _loop_sim(self):
        t0 = time.monotonic()
        while self._running:
            t = time.monotonic() - t0
            roll = 30.0 * math.sin(2 * math.pi * 0.10 * t)
            pitch = 15.0 * math.sin(2 * math.pi * 0.07 * t)
            self._push(quat_from_rpy_deg(roll, pitch))
            time.sleep(0.01)

    def _loop_serial(self):
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(64)
            except Exception as exc:
                self.status = f"read error: {type(exc).__name__}: {exc}"
                self.connected = False
                return
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                sample = parse_line(raw)
                if sample is not None:
                    self._push(sample.quat)

    # -- consumer ----------------------------------------------------------
    def latest(self):
        return self._samples


def interpolated_attitude(samples, now, max_age=0.5):
    """Slerp-interpolated quaternion at time `now`, or None if stale/empty."""
    if not samples:
        return None
    if now - samples[-1][0] > max_age:
        return None                       # stream stale / dead
    if now <= samples[0][0]:
        return samples[0][1]
    for i in range(len(samples) - 1, 0, -1):
        t0, q0 = samples[i - 1]
        t1, q1 = samples[i]
        if t0 <= now:
            if t1 <= t0 or now >= t1:
                return q1
            return slerp(q0, q1, (now - t0) / (t1 - t0))
    return samples[-1][1]


def list_ports():
    ports = list(_list_ports.comports())
    if not ports:
        print("no serial ports found")
        return 1
    for p in ports:
        print(f"{p.device:24s} {p.description}")
    return 0


def numpy_pipeline_check(seconds):
    """Headless validation: parse synthetic lines + quat math for N seconds."""
    t0 = time.monotonic()
    n = 0
    print(f"[headless] validating parser/quat pipeline for {seconds:.0f}s")
    while time.monotonic() - t0 < seconds:
        line = ("G: -0.09 0.03 -0.13 rad | A:0.004 0.128 0.992 g | "
                "Q: 0.097 0.001 -0.064 0.993 | RPY: 7.3 -0.8 -169.0 deg")
        s = parse_line(line)
        if s is not None:
            quat_to_rpy_deg(s.quat)
            n += 1
        time.sleep(0.005)
    rate = n / (time.monotonic() - t0)
    print(f"[headless] parsed {n} samples ({rate:.0f}/s) — OK")
    return 0


# ---------------------------------------------------------------------------
# Geometry helpers (pure, GL-free)
# ---------------------------------------------------------------------------

def ball_to_screen(cx, cy, roll_deg, pitch_deg, px_per_deg, lx, ly):
    r = math.radians(roll_deg)
    x = lx
    y = ly - pitch_deg * px_per_deg
    return (cx + x * math.cos(r) - y * math.sin(r),
            cy + x * math.sin(r) + y * math.cos(r))


def bank_scale_pos(cx, cy, radius, bank_deg):
    b = math.radians(bank_deg)
    return (cx + radius * math.sin(b), cy + radius * math.cos(b))


# ---------------------------------------------------------------------------
# pyglet renderer
# ---------------------------------------------------------------------------

C_BG = (0.08, 0.09, 0.12)
C_FACEPLATE = (0.13, 0.14, 0.16)
C_BEZEL = (0.30, 0.31, 0.34)
C_SKY = (0.16, 0.46, 0.76)
C_GROUND = (0.48, 0.30, 0.13)
C_HORIZON = (1.0, 1.0, 1.0)
C_LADDER = (1.0, 1.0, 1.0)
C_SCALE = (0.92, 0.92, 0.92)
C_POINTER = (1.0, 1.0, 1.0)
C_SYMBOL = (1.0, 0.84, 0.10)

PITCH_LADDER_DEG = (-60, -50, -40, -30, -20, -10, 10, 20, 30, 40, 50, 60)
PITCH_STUB_DEG = (-5, 5)
BANK_TICKS_DEG = (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60)


def run_app(args) -> int:
    try:
        import pyglet
        from pyglet import gl
        from pyglet.window import key
    except Exception as exc:
        print(f"[headless] pyglet/GL unavailable ({type(exc).__name__}: {exc})")
        if args.headless_test:
            return numpy_pipeline_check(args.headless_test)
        print("Cannot initialize pyglet/GL.", file=sys.stderr)
        return 1

    reader = TelemetryReader(args.port, args.baud, sim=args.sim)
    reader.start()

    def _circle_fan(cx, cy, r, segments=72):
        gl.glBegin(gl.GL_TRIANGLE_FAN)
        gl.glVertex2f(cx, cy)
        for i in range(segments + 1):
            a = 2.0 * math.pi * i / segments
            gl.glVertex2f(cx + r * math.cos(a), cy + r * math.sin(a))
        gl.glEnd()

    class AttitudeWindow(pyglet.window.Window):
        def __init__(self):
            config = gl.Config(double_buffer=True, stencil_size=8)
            super().__init__(width=args.width, height=args.height,
                             caption="IMU Attitude Indicator (fixed)",
                             resizable=True,
                             visible=not args.headless_test, config=config)
            try:
                self.set_vsync(True)
            except Exception:
                pass
            self.roll = self.pitch = self.yaw = 0.0
            self.have_attitude = False
            self._fps = 0.0
            self._fps_last = time.monotonic()
            self._fps_frames = 0
            fs = max(9, int(min(self.width, self.height) * 0.022))
            self.hud = pyglet.text.Label(
                "", font_name="Monospace", font_size=11, x=10,
                y=self.height - 10, anchor_y="top", multiline=True,
                width=self.width - 20, color=(230, 230, 230, 255))
            self.help_label = pyglet.text.Label(
                "ESC: quit", font_name="Monospace", font_size=10, x=10,
                y=10, anchor_y="bottom", color=(150, 150, 150, 255))
            self.ladder_labels = {}
            for p in PITCH_LADDER_DEG:
                for side in ("L", "R"):
                    self.ladder_labels[(p, side)] = pyglet.text.Label(
                        str(abs(p)), font_name="Monospace", font_size=fs,
                        anchor_x="center", anchor_y="center",
                        color=(255, 255, 255, 255))

        def _geometry(self):
            cx, cy = self.width / 2.0, self.height / 2.0
            R = 0.40 * min(self.width, self.height)
            return cx, cy, R, R / 45.0

        def _draw_faceplate(self, cx, cy, R):
            gl.glColor3f(*C_FACEPLATE); _circle_fan(cx, cy, R * 1.16)
            gl.glColor3f(*C_BEZEL);     _circle_fan(cx, cy, R * 1.14)
            gl.glColor3f(*C_FACEPLATE); _circle_fan(cx, cy, R * 1.02)

        def _draw_ball(self, cx, cy, R, ppd):
            gl.glEnable(gl.GL_STENCIL_TEST)
            gl.glStencilFunc(gl.GL_ALWAYS, 1, 0xFF)
            gl.glStencilOp(gl.GL_KEEP, gl.GL_KEEP, gl.GL_REPLACE)
            gl.glStencilMask(0xFF)
            gl.glColorMask(False, False, False, False)
            _circle_fan(cx, cy, R)
            gl.glColorMask(True, True, True, True)
            gl.glStencilFunc(gl.GL_EQUAL, 1, 0xFF)
            gl.glStencilMask(0x00)
            gl.glPushMatrix()
            gl.glTranslatef(cx, cy, 0.0)
            gl.glRotatef(self.roll, 0.0, 0.0, 1.0)
            gl.glTranslatef(0.0, -self.pitch * ppd, 0.0)
            B = R * 2.2
            gl.glBegin(gl.GL_QUADS)
            gl.glColor3f(*C_SKY)
            gl.glVertex2f(-B, 0.0); gl.glVertex2f(B, 0.0)
            gl.glVertex2f(B, B);    gl.glVertex2f(-B, B)
            gl.glColor3f(*C_GROUND)
            gl.glVertex2f(-B, 0.0); gl.glVertex2f(B, 0.0)
            gl.glVertex2f(B, -B);   gl.glVertex2f(-B, -B)
            gl.glEnd()
            gl.glColor3f(*C_HORIZON)
            gl.glLineWidth(3.0)
            gl.glBegin(gl.GL_LINES)
            gl.glVertex2f(-B, 0.0); gl.glVertex2f(B, 0.0)
            gl.glEnd()
            gl.glColor3f(*C_LADDER)
            gl.glLineWidth(2.0)
            gl.glBegin(gl.GL_LINES)
            for p in PITCH_LADDER_DEG:
                half = R * (0.38 if p % 20 == 0 else 0.28)
                y = p * ppd
                gl.glVertex2f(-half, y); gl.glVertex2f(half, y)
            for p in PITCH_STUB_DEG:
                y = p * ppd
                gl.glVertex2f(-R * 0.13, y); gl.glVertex2f(R * 0.13, y)
            gl.glEnd()
            gl.glLineWidth(1.0)
            gl.glPopMatrix()
            gl.glDisable(gl.GL_STENCIL_TEST)

        def _draw_bank_scale(self, cx, cy, R):
            r_in, r_out = R * 0.97, R * 1.10
            gl.glColor3f(*C_SCALE)
            gl.glLineWidth(2.0)
            gl.glBegin(gl.GL_LINES)
            for b in BANK_TICKS_DEG:
                if b == 0:
                    continue
                long = b % 30 == 0
                r1 = r_in if long else (r_in + r_out) / 2.0 + R * 0.02
                x1, y1 = bank_scale_pos(cx, cy, r1, b)
                x2, y2 = bank_scale_pos(cx, cy, r_out, b)
                gl.glVertex2f(x1, y1); gl.glVertex2f(x2, y2)
            gl.glEnd()
            xt, yt = bank_scale_pos(cx, cy, r_out + R * 0.02, 0.0)
            s = R * 0.045
            gl.glBegin(gl.GL_TRIANGLES)
            gl.glVertex2f(xt, yt + s)
            gl.glVertex2f(xt - s * 0.8, yt)
            gl.glVertex2f(xt + s * 0.8, yt)
            gl.glEnd()
            gl.glLineWidth(1.0)

        def _draw_roll_pointer(self, cx, cy, R):
            gl.glPushMatrix()
            gl.glTranslatef(cx, cy, 0.0)
            gl.glRotatef(-self.roll, 0.0, 0.0, 1.0)
            r_tip, r_base = R * 0.97, R * 0.82
            w = R * 0.05
            gl.glColor3f(*C_POINTER)
            gl.glBegin(gl.GL_TRIANGLES)
            gl.glVertex2f(0.0, r_tip)
            gl.glVertex2f(-w, r_base)
            gl.glVertex2f(w, r_base)
            gl.glEnd()
            gl.glPopMatrix()

        def _draw_aircraft_symbol(self, cx, cy, R):
            gl.glColor3f(*C_SYMBOL)
            w_out, w_in = R * 0.55, R * 0.09
            h = R * 0.035
            gl.glBegin(gl.GL_QUADS)
            for sgn in (-1.0, 1.0):
                x0, x1 = cx + sgn * w_in, cx + sgn * w_out
                gl.glVertex2f(x0, cy - h); gl.glVertex2f(x1, cy - h)
                gl.glVertex2f(x1, cy + h); gl.glVertex2f(x0, cy + h)
            d = R * 0.045
            gl.glVertex2f(cx - d, cy - d); gl.glVertex2f(cx + d, cy - d)
            gl.glVertex2f(cx + d, cy + d); gl.glVertex2f(cx - d, cy + d)
            gl.glEnd()

        def _update_ladder_labels(self, cx, cy, R, ppd):
            for (p, side), lbl in self.ladder_labels.items():
                half = R * (0.38 if p % 20 == 0 else 0.28)
                lx = (half + R * 0.10) * (1.0 if side == "R" else -1.0)
                x, y = ball_to_screen(cx, cy, self.roll, self.pitch,
                                      ppd, lx, p * ppd)
                lbl.x, lbl.y = x, y
                lbl.draw()

        def _draw_hud(self):
            lines = [
                f"Port:  {'SIM' if reader.sim else (args.port or 'auto')}  "
                f"({'OK' if reader.connected else 'DOWN'})  "
                f"{reader.rate_hz:5.1f} Hz  render {self._fps:4.0f} FPS",
            ]
            if self.have_attitude:
                lines.append(f"Roll {self.roll:+6.1f}  Pitch {self.pitch:+6.1f}  "
                             f"Yaw {self.yaw:+6.1f} deg")
            else:
                lines.append(f"Status: {reader.status} — waiting for data...")
            self.hud.text = "\n".join(lines)
            self.hud.y = self.height - 10
            self.hud.width = max(self.width - 20, 50)
            self.hud.draw()
            self.help_label.draw()

        def on_draw(self):
            gl.glClearColor(*C_BG, 1.0)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT
                       | gl.GL_STENCIL_BUFFER_BIT)
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glLoadIdentity()
            gl.glOrtho(0, self.width, 0, self.height, -1, 1)
            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glLoadIdentity()
            gl.glDisable(gl.GL_DEPTH_TEST)
            gl.glEnable(gl.GL_LINE_SMOOTH)
            cx, cy, R, ppd = self._geometry()
            self._draw_faceplate(cx, cy, R)
            self._draw_ball(cx, cy, R, ppd)
            self._draw_bank_scale(cx, cy, R)
            self._draw_roll_pointer(cx, cy, R)
            self._draw_aircraft_symbol(cx, cy, R)
            self._update_ladder_labels(cx, cy, R, ppd)
            self._draw_hud()

        def update(self, dt):
            now = time.monotonic()
            q = interpolated_attitude(reader.latest(), now)
            if q is not None:
                self.roll, self.pitch, self.yaw = quat_to_rpy_deg(q)
                self.have_attitude = True
            self._fps_frames += 1
            if now - self._fps_last >= 1.0:
                self._fps = self._fps_frames / (now - self._fps_last)
                self._fps_frames = 0
                self._fps_last = now
            self.invalid = True

        def on_key_press(self, symbol, modifiers):
            if symbol == key.ESCAPE:
                self.close()
                return pyglet.event.EVENT_HANDLED

        def on_resize(self, width, height):
            gl.glViewport(0, 0, max(width, 1), max(height, 1))
            return pyglet.event.EVENT_HANDLED

        def on_close(self):
            reader.stop()
            super().on_close()

    try:
        window = AttitudeWindow()
    except Exception as exc:
        reader.stop()
        print(f"[headless] GL window unavailable ({type(exc).__name__}: {exc})")
        if args.headless_test:
            return numpy_pipeline_check(args.headless_test)
        print("Cannot open a GL window.", file=sys.stderr)
        return 1

    pyglet.clock.schedule_interval(window.update, 1.0 / 60.0)
    if args.headless_test:
        def _finish(dt):
            pyglet.app.exit()
        pyglet.clock.schedule_once(_finish, args.headless_test)
    pyglet.app.run()
    reader.stop()
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IMU attitude indicator (fixed)")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=640)
    ap.add_argument("--headless-test", type=float, metavar="SECONDS")
    args = ap.parse_args(argv)
    if args.list:
        return list_ports()
    if not args.sim and not args.port:
        print("error: provide --port or use --sim", file=sys.stderr)
        return 2
    return run_app(args)


if __name__ == "__main__":
    sys.exit(main())
