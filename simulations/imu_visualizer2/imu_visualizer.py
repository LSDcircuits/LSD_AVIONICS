#!/usr/bin/env python3
"""IMU 3D attitude visualizer.

Connects to an RP2040 (LSM6DS3 + on-chip Mahony filter) streaming telemetry
over USB CDC serial, parses the quaternion (w, x, y, z; body-relative-to-NED)
and renders a rigid-body aircraft rotating in real time with pyglet 1.5
(legacy OpenGL, no Qt).

Usage:
    python imu_visualizer.py [--port COMx|/dev/ttyACM0] [--baud 115200]
                             [--list] [--sim] [--width 960 --height 720]
                             [--headless-test SECONDS]

Controls:
    Arrow keys  orbit camera
    r           reset camera view
    t           tare yaw (make current heading the new "north")
    ESC         quit
"""

from __future__ import annotations

import argparse
import collections
import math
import sys
import threading
import time

from serial_parser import ImuSample, parse_line
from quat_math import quat_to_mat3, slerp, ned_to_gl, _NED_TO_GL

# ---------------------------------------------------------------------------
# Telemetry reader thread
# ---------------------------------------------------------------------------


class TelemetryReader(threading.Thread):
    """Background thread reading telemetry lines.

    Real mode: pyserial, blocking readline(), reconnecting every 1 s on
    serial errors. Sim mode: SimSerial (same readline() interface), so the
    parse/slerp/render path is identical.

    Parsed samples are appended to a deque(maxlen=8) protected by a lock;
    each entry is (sample, arrival_wall_time) so the render thread can
    interpolate between the last two samples.
    """

    def __init__(self, port: str | None, baud: int, sim: bool = False):
        super().__init__(daemon=True, name="telemetry-reader")
        self.port = port
        self.baud = baud
        self.sim = sim
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self.samples: collections.deque = collections.deque(maxlen=8)
        # HUD-facing status
        self.status = "starting"
        self.connected = False
        self.rate_hz = 0.0
        self._rate_window: collections.deque = collections.deque(maxlen=64)

    # -- public API ------------------------------------------------------
    def stop(self):
        self._stop_event.set()

    def latest(self):
        with self._lock:
            return list(self.samples)

    # -- internals -------------------------------------------------------
    def _open(self):
        """Open the serial source (real or simulated)."""
        if self.sim:
            from sim_serial import SimSerial

            self.status = "simulated stream"
            return SimSerial()
        import serial

        return serial.Serial(self.port, self.baud, timeout=1.0)

    def _record(self, sample: ImuSample):
        now = time.monotonic()
        with self._lock:
            self.samples.append((sample, now))
        self._rate_window.append(now)
        if len(self._rate_window) >= 2:
            span = self._rate_window[-1] - self._rate_window[0]
            if span > 1e-6:
                self.rate_hz = (len(self._rate_window) - 1) / span

    def run(self):
        ser = None
        while not self._stop_event.is_set():
            try:
                if ser is None:
                    self.status = (
                        f"connecting {self.port} @ {self.baud}"
                        if not self.sim
                        else "starting sim"
                    )
                    self.connected = False
                    ser = self._open()
                    self.connected = True
                    if not self.sim:
                        self.status = f"connected {self.port}"

                raw = ser.readline()
                if not raw:
                    # readline timeout; just loop again
                    continue
                try:
                    line = raw.decode("ascii", errors="replace")
                except Exception:
                    continue
                sample = parse_line(line)
                if sample is not None:
                    self._record(sample)
                    self.status = (
                        "simulated stream" if self.sim else f"receiving {self.port}"
                    )
                # Non-telemetry lines (boot/diagnostics) are ignored silently.

            except Exception as exc:  # serial.SerialException, OSError, ...
                self.connected = False
                self.status = f"serial error: {exc}; retrying in 1 s"
                try:
                    if ser is not None:
                        ser.close()
                except Exception:
                    pass
                ser = None
                # Retry reconnect every 1 s (interruptible by stop()).
                self._stop_event.wait(1.0)

        try:
            if ser is not None:
                ser.close()
        except Exception:
            pass
        self.connected = False
        self.status = "stopped"


# ---------------------------------------------------------------------------
# GL-independent frame logic (kept importable/testable without pyglet)
# ---------------------------------------------------------------------------


def interpolated_attitude(samples, now: float):
    """Pick the render quaternion from the sample buffer.

    Slerps between the last two quaternion samples. The interpolation
    parameter advances with wall-clock time past the newest sample, scaled
    by the sample period taken from the HostUS timestamps (falling back to
    arrival wall times if HostUS does not advance). Returns
    (qw, qx, qy, qz) or None if no sample is available yet.
    """
    if not samples:
        return None
    latest_sample, latest_wall = samples[-1]
    if len(samples) == 1:
        return (latest_sample.qw, latest_sample.qx, latest_sample.qy, latest_sample.qz)

    prev_sample, prev_wall = samples[-2]
    # Sample period from firmware timestamps (microseconds).
    period = (latest_sample.host_us - prev_sample.host_us) / 1e6
    if period <= 1e-6 or period > 1.0:
        period = max(latest_wall - prev_wall, 1e-3)
    t = (now - latest_wall) / period
    q0 = (prev_sample.qw, prev_sample.qx, prev_sample.qy, prev_sample.qz)
    q1 = (latest_sample.qw, latest_sample.qx, latest_sample.qy, latest_sample.qz)
    return slerp(q0, q1, t)


def yaw_of_quat(q) -> float:
    """Extract yaw (rotation about NED Down axis), radians."""
    R = quat_to_mat3(*q)
    return math.atan2(R[1, 0], R[0, 0])


def yaw_tare_quat(q):
    """Quaternion which, pre-multiplied with q, cancels its yaw component."""
    psi = yaw_of_quat(q)
    half = -psi / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def quat_mul(a, b):
    """Hamilton product (w, x, y, z). a*b applies b first, then a."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def numpy_pipeline_check(seconds: float = 1.0) -> int:
    """Exercise the full non-GL pipeline on synthetic data.

    Used by --headless-test when no GL context can be created (e.g. CI
    sandbox without display/EGL): validates that simulated telemetry parses,
    interpolates and converts to a valid GL matrix. Loops sim->parse->slerp
    ->matrix iterations until the requested number of seconds has elapsed,
    matching the duration contract of the GL offscreen path.
    """
    import numpy as np
    from sim_serial import SimSerial

    sim = SimSerial()
    samples = []
    t0 = time.monotonic()
    deadline = t0 + max(float(seconds), 0.0)
    iterations = 0
    det = float("nan")
    while True:
        line = sim.readline().decode()
        s = parse_line(line)
        assert s is not None, f"sim line failed to parse: {line!r}"
        samples.append((s, time.monotonic()))
        samples = samples[-10:]
        if len(samples) >= 2:
            # With a single sample interpolated_attitude returns the raw
            # parsed quaternion (not unit-norm due to 3-decimal rounding);
            # only the slerp path yields a normalized quaternion.
            q = interpolated_attitude(samples, time.monotonic())
            assert q is not None
            n = math.sqrt(sum(v * v for v in q))
            assert abs(n - 1.0) < 1e-6, f"non-unit quaternion after slerp: {n}"
            m = ned_to_gl(quat_to_mat3(*q))
            M = np.array(m, dtype=float).reshape(4, 4, order="F")
            det = np.linalg.det(M[:3, :3])
            assert abs(det - 1.0) < 1e-5, f"bad GL matrix determinant: {det}"
            assert abs(M[3, 3] - 1.0) < 1e-6
        iterations += 1
        # Always run at least two iterations so the slerp/matrix path is
        # exercised even for a very small requested duration.
        if iterations >= 2 and time.monotonic() >= deadline:
            break
    elapsed = time.monotonic() - t0
    print(
        f"[headless] numpy-only pipeline check OK: {iterations} "
        f"sim->parse->slerp->matrix iterations in {elapsed:.2f}s "
        f"(requested {float(seconds):.2f}s, det={det:.6f})"
    )
    return 0


# ---------------------------------------------------------------------------
# pyglet renderer (imported lazily so non-GL tests never touch it)
# ---------------------------------------------------------------------------


def _build_aircraft_vertices():
    """Triangle vertices for a simple aircraft model.

    Geometry is authored in NED body coordinates (x = nose/forward,
    y = right wing, z = down) and then mapped through _NED_TO_GL so the
    model space matches the camera convention (nose into the screen at
    identity attitude).
    """
    import numpy as np

    M = _NED_TO_GL

    def v(x, y, z):
        return tuple((M @ np.array([x, y, z], dtype=float)).tolist())

    # Key points (body NED coords)
    NOSE = v(1.5, 0.0, 0.0)
    TAIL = v(-1.1, 0.0, 0.0)
    BODY_T = v(0.0, 0.0, -0.22)  # fuselage top
    BODY_B = v(0.0, 0.0, 0.22)  # fuselage bottom
    BODY_L = v(0.0, -0.22, 0.0)
    BODY_R = v(0.0, 0.22, 0.0)
    WING_L = v(0.1, -1.7, 0.0)
    WING_R = v(0.1, 1.7, 0.0)
    WING_L_BACK = v(-0.55, -1.4, 0.0)
    WING_R_BACK = v(-0.55, 1.4, 0.0)
    FIN_TOP = v(-1.15, 0.0, -0.75)  # vertical stabilizer (up = -Down)
    FIN_BASE = v(-0.75, 0.0, 0.0)
    TAIL_L = v(-1.0, -0.65, 0.0)
    TAIL_R = v(-1.0, 0.65, 0.0)
    TAIL_MID = v(-0.7, 0.0, 0.0)

    # (vertices, color) triangles: flat-shaded quads split into triangles.
    fuselage_col = (0.75, 0.75, 0.8)
    wing_col = (0.9, 0.45, 0.15)
    tail_col = (0.2, 0.55, 0.9)
    fin_col = (0.85, 0.2, 0.25)

    tris = []

    def tri(a, b, c, col):
        tris.append((col, [a, b, c]))

    # Fuselage: 4-sided pyramid-ish tube nose->tail.
    for a, b in ((BODY_T, BODY_R), (BODY_R, BODY_B), (BODY_B, BODY_L), (BODY_L, BODY_T)):
        tri(NOSE, a, b, fuselage_col)
        tri(TAIL, b, a, tuple(c * 0.8 for c in fuselage_col))
    # Wings (thin double-sided triangles, drawn twice with flip).
    tri(v(0.45, 0.0, 0.0), WING_L, WING_L_BACK, wing_col)
    tri(v(0.45, 0.0, 0.0), WING_L_BACK, v(-0.45, 0.0, 0.0), wing_col)
    tri(v(0.45, 0.0, 0.0), WING_R_BACK, WING_R, wing_col)
    tri(v(0.45, 0.0, 0.0), v(-0.45, 0.0, 0.0), WING_R_BACK, wing_col)
    # Horizontal stabilizer.
    tri(TAIL_MID, TAIL_L, TAIL, tail_col)
    tri(TAIL_MID, TAIL, TAIL_R, tail_col)
    # Vertical stabilizer.
    tri(FIN_BASE, FIN_TOP, TAIL, fin_col)

    return tris


def run_app(args) -> int:
    """Create the window and run the pyglet event loop.

    Any failure to import pyglet's GL bindings or create a GL context
    (headless CI sandbox without display/EGL/GLU) is caught: in
    --headless-test mode we fall back to the numpy-only pipeline check and
    exit 0; otherwise we report the problem and exit 1. Never a crash.
    """
    try:
        import pyglet
        from pyglet import gl
        from pyglet.window import key
    except Exception as exc:
        print(f"[headless] pyglet/GL unavailable ({type(exc).__name__}: {exc})")
        if args.headless_test:
            print("[headless] falling back to numpy-only pipeline validation")
            return numpy_pipeline_check(args.headless_test)
        print("Cannot initialize pyglet/GL. Use --headless-test for CI, or "
              "run on a machine with a display.", file=sys.stderr)
        return 1

    reader = TelemetryReader(args.port, args.baud, sim=args.sim)
    reader.start()

    aircraft_tris = _build_aircraft_vertices()

    class ImuWindow(pyglet.window.Window):
        def __init__(self):
            config = gl.Config(double_buffer=True, depth_size=24)
            super().__init__(
                width=args.width,
                height=args.height,
                caption="IMU Attitude Visualizer",
                resizable=True,
                visible=not args.headless_test,
                config=config,
            )
            try:
                self.set_vsync(True)
            except Exception:
                pass  # vsync not controllable on some drivers; non-fatal
            # Camera orbit state.
            self.cam_azimuth = 0.0  # deg, about +Y_gl (up)
            self.cam_elevation = 20.0  # deg, looking slightly down at body
            self.cam_distance = 6.0
            # Tare: pre-rotation cancelling yaw at the moment 't' is pressed.
            self.tare_q = (1.0, 0.0, 0.0, 0.0)
            self.current_q = (1.0, 0.0, 0.0, 0.0)
            self._fps = 0.0
            self._fps_last = time.monotonic()
            self._fps_frames = 0

            self.hud = pyglet.text.Label(
                "", font_name="Monospace", font_size=11,
                x=10, y=self.height - 10, anchor_y="top",
                multiline=True, width=self.width - 20,
                color=(230, 230, 230, 255),
            )
            self.help_label = pyglet.text.Label(
                "arrows: orbit   r: reset view   t: tare yaw   ESC: quit",
                font_name="Monospace", font_size=10,
                x=10, y=10, anchor_y="bottom",
                color=(150, 150, 150, 255),
            )

        # -- camera / scene helpers -------------------------------------
        def _set_perspective(self):
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glLoadIdentity()
            aspect = max(self.width, 1) / max(self.height, 1)
            fovy, near, far = 45.0, 0.1, 100.0
            f = 1.0 / math.tan(math.radians(fovy) / 2.0)
            # Column-major frustum matrix (gluPerspective equivalent).
            m = (gl.GLfloat * 16)(
                f / aspect, 0, 0, 0,
                0, f, 0, 0,
                0, 0, (far + near) / (near - far), -1,
                0, 0, 2 * far * near / (near - far), 0,
            )
            gl.glLoadMatrixf(m)
            gl.glMatrixMode(gl.GL_MODELVIEW)

        def _set_camera(self):
            gl.glLoadIdentity()
            # Camera orbits the origin: south of the body looking north,
            # elevated; arrow keys adjust azimuth/elevation.
            gl.glTranslatef(0.0, 0.0, -self.cam_distance)
            gl.glRotatef(-self.cam_elevation, 1.0, 0.0, 0.0)
            gl.glRotatef(-self.cam_azimuth, 0.0, 1.0, 0.0)

        def _draw_world_frame(self):
            """NED world axes + floor grid, in camera (GL) coordinates."""
            gl.glLineWidth(2.0)
            gl.glBegin(gl.GL_LINES)
            # N (red, forward): NED (1,0,0) -> GL (0,0,-1)
            gl.glColor3f(0.9, 0.2, 0.2)
            gl.glVertex3f(0, 0, 0)
            gl.glVertex3f(0, 0, -3.0)
            # E (green, right): NED (0,1,0) -> GL (1,0,0)
            gl.glColor3f(0.2, 0.8, 0.2)
            gl.glVertex3f(0, 0, 0)
            gl.glVertex3f(3.0, 0, 0)
            # D (blue, down): NED (0,0,1) -> GL (0,-1,0)
            gl.glColor3f(0.3, 0.4, 0.95)
            gl.glVertex3f(0, 0, 0)
            gl.glVertex3f(0, -3.0, 0)
            gl.glEnd()

            # Floor grid on the N-E plane at Down = +2 (GL y = -2).
            gl.glLineWidth(1.0)
            gl.glColor4f(0.35, 0.35, 0.35, 1.0)
            gl.glBegin(gl.GL_LINES)
            y = -2.0
            for i in range(-4, 5):
                gl.glVertex3f(i * 1.0, y, -4.0)
                gl.glVertex3f(i * 1.0, y, 4.0)
                gl.glVertex3f(-4.0, y, i * 1.0)
                gl.glVertex3f(4.0, y, i * 1.0)
            gl.glEnd()

        def _draw_body(self):
            """Aircraft model + body axes, rotated by the attitude matrix."""
            q_disp = quat_mul(self.tare_q, self.current_q)
            m = ned_to_gl(quat_to_mat3(*q_disp))
            gl.glPushMatrix()
            gl.glMultMatrixf(m)

            gl.glBegin(gl.GL_TRIANGLES)
            for col, verts in aircraft_tris:
                gl.glColor3f(*col)
                for vx, vy, vz in verts:
                    gl.glVertex3f(vx, vy, vz)
            gl.glEnd()

            # Body axes (colored so rotation direction is unambiguous):
            # x_body (fwd) red, y_body (right) green, z_body (down) blue.
            gl.glLineWidth(3.0)
            gl.glBegin(gl.GL_LINES)
            for axis, col in (
                ((0, 0, -2.2), (1.0, 0.1, 0.1)),  # fwd = -Z in model space
                ((2.2, 0, 0), (0.1, 1.0, 0.1)),  # right = +X
                ((0, -2.2, 0), (0.2, 0.3, 1.0)),  # down = -Y
            ):
                gl.glColor3f(*col)
                gl.glVertex3f(0, 0, 0)
                gl.glVertex3f(*axis)
            gl.glEnd()
            gl.glLineWidth(1.0)
            gl.glPopMatrix()

        def _draw_hud(self, lines):
            self.hud.text = "\n".join(lines)
            self.hud.y = self.height - 10
            self.hud.width = max(self.width - 20, 50)
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glPushMatrix()
            gl.glLoadIdentity()
            gl.glOrtho(0, self.width, 0, self.height, -1, 1)
            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glPushMatrix()
            gl.glLoadIdentity()
            gl.glDisable(gl.GL_DEPTH_TEST)
            gl.glDisable(gl.GL_LIGHTING)
            self.hud.draw()
            self.help_label.draw()
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glPopMatrix()
            gl.glMatrixMode(gl.GL_PROJECTION)
            gl.glPopMatrix()
            gl.glMatrixMode(gl.GL_MODELVIEW)

        # -- pyglet events ----------------------------------------------
        def on_draw(self):
            gl.glClearColor(0.08, 0.09, 0.12, 1.0)
            self.clear()
            gl.glEnable(gl.GL_DEPTH_TEST)
            gl.glEnable(gl.GL_LINE_SMOOTH)
            self._set_perspective()
            self._set_camera()
            self._draw_world_frame()
            self._draw_body()

            samples = reader.latest()
            if samples:
                s = samples[-1][0]
                qw, qx, qy, qz = quat_mul(self.tare_q, self.current_q)
                hud_lines = [
                    f"Port:  {'SIM' if reader.sim else (args.port or 'auto')}  "
                    f"({'OK' if reader.connected else 'DOWN'})  {reader.rate_hz:5.1f} Hz  "
                    f"render {self._fps:4.0f} FPS",
                    f"Q:     {qw:+.3f} {qx:+.3f} {qy:+.3f} {qz:+.3f}  (w x y z, body->NED)",
                    f"RPY:   {s.roll:+6.1f} {s.pitch:+6.1f} {s.yaw:+6.1f} deg",
                    f"Gyro:  {s.gx:+7.2f} {s.gy:+7.2f} {s.gz:+7.2f} dps",
                    f"Accel: {s.ax:+.3f} {s.ay:+.3f} {s.az:+.3f} g",
                    f"HostUS: {s.host_us}",
                    f"Status: {reader.status}",
                ]
            else:
                hud_lines = [
                    f"Port: {'SIM' if reader.sim else (args.port or 'auto')}",
                    f"Status: {reader.status} — waiting for data…",
                ]
            self._draw_hud(hud_lines)

        def update(self, dt):
            now = time.monotonic()
            q = interpolated_attitude(reader.latest(), now)
            if q is not None:
                self.current_q = q
            # FPS counter (1 s window).
            self._fps_frames += 1
            if now - self._fps_last >= 1.0:
                self._fps = self._fps_frames / (now - self._fps_last)
                self._fps_frames = 0
                self._fps_last = now
            self.invalid = True  # keep redrawing (also drives on_draw)

        def on_key_press(self, symbol, modifiers):
            if symbol == key.ESCAPE:
                self.close()
                return pyglet.event.EVENT_HANDLED
            elif symbol == key.LEFT:
                self.cam_azimuth -= 5.0
            elif symbol == key.RIGHT:
                self.cam_azimuth += 5.0
            elif symbol == key.UP:
                self.cam_elevation = min(self.cam_elevation + 5.0, 89.0)
            elif symbol == key.DOWN:
                self.cam_elevation = max(self.cam_elevation - 5.0, -89.0)
            elif symbol == key.R:
                self.cam_azimuth = 0.0
                self.cam_elevation = 20.0
            elif symbol == key.T:
                # Tare yaw: subtract the current yaw offset.
                self.tare_q = yaw_tare_quat(quat_mul(self.tare_q, self.current_q))

        def on_resize(self, width, height):
            gl.glViewport(0, 0, max(width, 1), max(height, 1))
            return pyglet.event.EVENT_HANDLED

        def on_close(self):
            reader.stop()
            super().on_close()

    try:
        window = ImuWindow()
    except Exception as exc:
        # No display / no GL context available (headless CI sandbox).
        reader.stop()
        print(f"[headless] GL window unavailable ({type(exc).__name__}: {exc})")
        if args.headless_test:
            print("[headless] falling back to numpy-only pipeline validation")
            return numpy_pipeline_check(args.headless_test)
        print("Cannot open a GL window. Use --headless-test for CI, or run "
              "on a machine with a display.", file=sys.stderr)
        return 1

    pyglet.clock.schedule_interval(window.update, 1.0 / 60.0)
    if args.headless_test:
        # Run the requested number of seconds offscreen, then exit 0.
        def _finish(dt):
            pyglet.app.exit()

        pyglet.clock.schedule_once(_finish, args.headless_test)
    pyglet.app.run()
    reader.stop()
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def list_ports() -> int:
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        print("pyserial not installed; cannot list ports", file=sys.stderr)
        return 1
    ports = list(lp.comports())
    if not ports:
        print("No serial ports found.")
    for p in ports:
        print(f"{p.device:20s} {p.description}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="IMU 3D attitude visualizer (Pico -> PC)")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200, help="baud rate (nominal for USB CDC)")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--sim", action="store_true", help="simulated telemetry, no hardware")
    ap.add_argument("--width", type=int, default=960)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument(
        "--headless-test",
        type=float,
        metavar="SECONDS",
        help="run for SECONDS then exit 0 (CI smoke test; without a GL "
             "context, falls back to a numpy-only pipeline loop for the "
             "same duration)",
    )
    args = ap.parse_args(argv)

    if args.list:
        return list_ports()
    if not args.sim and not args.port:
        print("error: provide --port or use --sim (see --list for ports)", file=sys.stderr)
        return 2
    return run_app(args)


if __name__ == "__main__":
    sys.exit(main())
