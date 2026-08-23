#!/usr/bin/env python3
"""Aviation attitude indicator (artificial horizon) for the IMU telemetry set.

Connects to an RP2040 (LSM6DS3 + on-chip Mahony filter) streaming telemetry
over USB CDC serial — the exact same stream as imu_visualizer.py — and
renders a classic primary-flight-display attitude indicator: sky/ground ball
with pitch ladder, fixed bank scale, roll pointer and fixed aircraft symbol.
Driven by the streamed quaternion (w, x, y, z; body-relative-to-NED),
slerp-interpolated for smooth motion.

Requires the other modules of this project (serial_parser, quat_math,
sim_serial, imu_visualizer) to be importable in the same directory.
Renderer: pyglet 1.5 legacy OpenGL (2D ortho + stencil clip, no Qt).

Usage:
    python attitude_indicator.py [--port COMx|/dev/ttyACM0] [--baud 115200]
                                 [--list] [--sim] [--width 640 --height 640]
                                 [--headless-test SECONDS]

Controls:
    ESC         quit

Display conventions (matches real AI behaviour):
    * Right roll (roll > 0): horizon slopes up to the right (ball drawn
      rotated +roll deg CCW), roll pointer swings clockwise toward the
      right-hand bank marks.
    * Pitch up (pitch > 0): horizon drops below the fixed aircraft symbol.
    * Pitch-ladder numbers are drawn upright (EFIS compromise) but ride the
      ball transform, so they track their bars exactly.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from serial_parser import ImuSample, parse_line  # noqa: F401  (re-exported path)
from quat_math import quat_to_mat3, slerp  # noqa: F401
from imu_visualizer import (
    TelemetryReader,
    interpolated_attitude,
    list_ports,
    numpy_pipeline_check,
)

# ---------------------------------------------------------------------------
# Pure (GL-free) attitude + geometry helpers — importable without pyglet
# ---------------------------------------------------------------------------


def quat_to_rpy_deg(q) -> tuple[float, float, float]:
    """(roll, pitch, yaw) in degrees from a (w, x, y, z) quaternion.

    NED frame, ZYX (yaw-pitch-roll) convention — identical to the firmware's
    quat_to_euler(): roll  = atan2(R[2,1], R[2,2]),
                     pitch = asin(-R[2,0]),
                     yaw   = atan2(R[1,0], R[0,0]).
    quat_to_mat3 normalizes internally, so 3-decimal printed quats are fine.
    """
    R = quat_to_mat3(*q)
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return roll, pitch, yaw


def ball_to_screen(cx, cy, roll_deg, pitch_deg, px_per_deg, lx, ly):
    """Map a ball-local point to screen coordinates.

    Ball-local frame: origin at ball centre (horizon), x right, y up; the
    +pitch ladder line sits at ly = pitch * px_per_deg. The on-screen
    transform mirrors imu_visualizer's GL calls:
        screen = (cx, cy) + Rot(+roll) @ (lx, ly - pitch * px_per_deg)
    i.e. right roll raises the right side of the horizon (CCW ball),
    pitch up drops the horizon below the symbol.
    """
    r = math.radians(roll_deg)
    x = lx
    y = ly - pitch_deg * px_per_deg
    return (
        cx + x * math.cos(r) - y * math.sin(r),
        cy + x * math.sin(r) + y * math.cos(r),
    )


def bank_scale_pos(cx, cy, radius, bank_deg):
    """Screen position of a bank-scale mark for bank angle `bank_deg`.

    0 deg is at 12 o'clock; positive (right) bank goes clockwise (screen
    right). Radial direction: (sin(bank), cos(bank)).
    """
    b = math.radians(bank_deg)
    return (cx + radius * math.sin(b), cy + radius * math.cos(b))


# ---------------------------------------------------------------------------
# pyglet renderer (imported lazily so non-GL tests never touch it)
# ---------------------------------------------------------------------------

# Instrument colours (r, g, b) in 0..1
C_BG = (0.08, 0.09, 0.12)
C_FACEPLATE = (0.13, 0.14, 0.16)
C_BEZEL = (0.30, 0.31, 0.34)
C_SKY = (0.16, 0.46, 0.76)
C_GROUND = (0.48, 0.30, 0.13)
C_HORIZON = (1.0, 1.0, 1.0)
C_LADDER = (1.0, 1.0, 1.0)
C_SCALE = (0.92, 0.92, 0.92)
C_POINTER = (1.0, 1.0, 1.0)
C_SYMBOL = (1.0, 0.84, 0.10)  # yellow aircraft symbol

PITCH_LADDER_DEG = (-60, -50, -40, -30, -20, -10, 10, 20, 30, 40, 50, 60)
PITCH_STUB_DEG = (-5, 5)  # short unlabelled stubs
BANK_TICKS_DEG = (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60)


def run_app(args) -> int:
    """Create the window and run the pyglet event loop.

    GL/context failures (headless machine) are caught: with --headless-test
    we fall back to imu_visualizer.numpy_pipeline_check() and exit 0;
    otherwise we print the problem and exit 1. Never a crash.
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
            super().__init__(
                width=args.width,
                height=args.height,
                caption="IMU Attitude Indicator",
                resizable=True,
                visible=not args.headless_test,
                config=config,
            )
            try:
                self.set_vsync(True)
            except Exception:
                pass  # vsync not controllable on some drivers; non-fatal

            # Current attitude (degrees); identity until first sample.
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.have_attitude = False
            self._fps = 0.0
            self._fps_last = time.monotonic()
            self._fps_frames = 0

            fs = max(9, int(min(self.width, self.height) * 0.022))
            self.hud = pyglet.text.Label(
                "", font_name="Monospace", font_size=11,
                x=10, y=self.height - 10, anchor_y="top",
                multiline=True, width=self.width - 20,
                color=(230, 230, 230, 255),
            )
            self.help_label = pyglet.text.Label(
                "ESC: quit", font_name="Monospace", font_size=10,
                x=10, y=10, anchor_y="bottom",
                color=(150, 150, 150, 255),
            )
            # One upright label per ladder value per side; repositioned every
            # frame onto the (rotated/translated) ladder bars.
            self.ladder_labels = {}
            for p in PITCH_LADDER_DEG:
                for side in ("L", "R"):
                    self.ladder_labels[(p, side)] = pyglet.text.Label(
                        str(abs(p)), font_name="Monospace", font_size=fs,
                        anchor_x="center", anchor_y="center",
                        color=(255, 255, 255, 255),
                    )

        # -- geometry ---------------------------------------------------
        def _geometry(self):
            """(cx, cy, R face radius, px_per_deg) for the current size."""
            cx, cy = self.width / 2.0, self.height / 2.0
            R = 0.40 * min(self.width, self.height)
            return cx, cy, R, R / 45.0  # 45 deg of pitch spans face radius

        # -- drawing ----------------------------------------------------
        def _draw_faceplate(self, cx, cy, R):
            gl.glColor3f(*C_FACEPLATE)
            _circle_fan(cx, cy, R * 1.16)
            gl.glColor3f(*C_BEZEL)
            _circle_fan(cx, cy, R * 1.14)
            gl.glColor3f(*C_FACEPLATE)
            _circle_fan(cx, cy, R * 1.02)

        def _draw_ball(self, cx, cy, R, ppd):
            """Sky/ground + horizon + pitch ladder, stencil-clipped to the
            instrument circle. Everything here is in the ball frame."""
            gl.glEnable(gl.GL_STENCIL_TEST)
            # 1) Write the instrument circle into the stencil buffer.
            gl.glStencilFunc(gl.GL_ALWAYS, 1, 0xFF)
            gl.glStencilOp(gl.GL_KEEP, gl.GL_KEEP, gl.GL_REPLACE)
            gl.glStencilMask(0xFF)
            gl.glColorMask(False, False, False, False)
            _circle_fan(cx, cy, R)
            gl.glColorMask(True, True, True, True)
            # 2) Draw the ball only where the circle was written.
            gl.glStencilFunc(gl.GL_EQUAL, 1, 0xFF)
            gl.glStencilMask(0x00)

            gl.glPushMatrix()
            gl.glTranslatef(cx, cy, 0.0)
            gl.glRotatef(self.roll, 0.0, 0.0, 1.0)   # right roll -> horizon right-up
            gl.glTranslatef(0.0, -self.pitch * ppd, 0.0)

            B = R * 2.2  # big enough to cover the circle at any pitch/roll
            gl.glBegin(gl.GL_QUADS)
            gl.glColor3f(*C_SKY)
            gl.glVertex2f(-B, 0.0); gl.glVertex2f(B, 0.0)
            gl.glVertex2f(B, B);    gl.glVertex2f(-B, B)
            gl.glColor3f(*C_GROUND)
            gl.glVertex2f(-B, 0.0); gl.glVertex2f(B, 0.0)
            gl.glVertex2f(B, -B);   gl.glVertex2f(-B, -B)
            gl.glEnd()

            # Horizon line.
            gl.glColor3f(*C_HORIZON)
            gl.glLineWidth(3.0)
            gl.glBegin(gl.GL_LINES)
            gl.glVertex2f(-B, 0.0); gl.glVertex2f(B, 0.0)
            gl.glEnd()

            # Pitch ladder.
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
            """Fixed tick arc on the bezel: 0 at top, +-60 deg."""
            r_in, r_out = R * 0.97, R * 1.10
            gl.glColor3f(*C_SCALE)
            gl.glLineWidth(2.0)
            gl.glBegin(gl.GL_LINES)
            for b in BANK_TICKS_DEG:
                if b == 0:
                    continue  # zero mark is the triangle below
                long = b % 30 == 0
                r1 = r_in if long else (r_in + r_out) / 2.0 + R * 0.02
                x1, y1 = bank_scale_pos(cx, cy, r1, b)
                x2, y2 = bank_scale_pos(cx, cy, r_out, b)
                gl.glVertex2f(x1, y1); gl.glVertex2f(x2, y2)
            gl.glEnd()
            # Zero-bank mark: small fixed triangle at 12 o'clock.
            xt, yt = bank_scale_pos(cx, cy, r_out + R * 0.02, 0.0)
            s = R * 0.045
            gl.glBegin(gl.GL_TRIANGLES)
            gl.glVertex2f(xt, yt + s)
            gl.glVertex2f(xt - s * 0.8, yt)
            gl.glVertex2f(xt + s * 0.8, yt)
            gl.glEnd()
            gl.glLineWidth(1.0)

        def _draw_roll_pointer(self, cx, cy, R):
            """Pointer riding the ball: swings clockwise for right roll."""
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
            """Fixed miniature aircraft: centre dot + two wing bars."""
            gl.glColor3f(*C_SYMBOL)
            w_out, w_in = R * 0.55, R * 0.09   # wing bar extents
            h = R * 0.035                       # bar half-height
            gl.glBegin(gl.GL_QUADS)
            for sgn in (-1.0, 1.0):
                x0, x1 = cx + sgn * w_in, cx + sgn * w_out
                gl.glVertex2f(x0, cy - h); gl.glVertex2f(x1, cy - h)
                gl.glVertex2f(x1, cy + h); gl.glVertex2f(x0, cy + h)
            d = R * 0.045  # centre dot
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
                lines.append(
                    f"Roll {self.roll:+6.1f}  Pitch {self.pitch:+6.1f}  "
                    f"Yaw {self.yaw:+6.1f} deg"
                )
            else:
                lines.append(f"Status: {reader.status} — waiting for data…")
            self.hud.text = "\n".join(lines)
            self.hud.y = self.height - 10
            self.hud.width = max(self.width - 20, 50)
            self.hud.draw()
            self.help_label.draw()

        # -- pyglet events ----------------------------------------------
        def on_draw(self):
            gl.glClearColor(*C_BG, 1.0)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT
                       | gl.GL_STENCIL_BUFFER_BIT)
            # 2D ortho over the whole window, y up.
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
            self.invalid = True  # keep redrawing (drives on_draw)

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
        # No display / no GL context available (headless machine).
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
        def _finish(dt):
            pyglet.app.exit()

        pyglet.clock.schedule_once(_finish, args.headless_test)
    pyglet.app.run()
    reader.stop()
    return 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Aviation attitude indicator (Pico IMU -> PC)")
    ap.add_argument("--port", help="serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="baud rate (nominal for USB CDC)")
    ap.add_argument("--list", action="store_true",
                    help="list serial ports and exit")
    ap.add_argument("--sim", action="store_true",
                    help="simulated telemetry, no hardware")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=640)
    ap.add_argument(
        "--headless-test",
        type=float,
        metavar="SECONDS",
        help="run for SECONDS then exit 0 (CI smoke test; without a GL "
             "context, falls back to the shared numpy-only pipeline loop "
             "for the same duration)",
    )
    args = ap.parse_args(argv)

    if args.list:
        return list_ports()
    if not args.sim and not args.port:
        print("error: provide --port or use --sim (see --list for ports)",
              file=sys.stderr)
        return 2
    return run_app(args)


if __name__ == "__main__":
    sys.exit(main())
