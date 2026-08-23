# IMU 3D Attitude Visualizer (Pico → PC)

Desktop Python app that connects to an RP2040 running LSM6DS3 + on-chip
Mahony-filter firmware over USB CDC serial, parses telemetry lines, and
renders a rigid-body aircraft rotating in real time from the streamed
quaternion. Renderer: **pyglet 1.5.x (legacy OpenGL, no Qt)**. Serial:
**pyserial** on a background reader thread. Smooth 60 FPS rendering via
slerp interpolation of ~10–100 Hz samples.

## Install

```bash
pip install -r requirements.txt   # pyserial, pyglet==1.5.*, numpy
```

Python 3.9+. On Linux you may need system GL libraries
(`libGL`, `libGLU` and an X/Wayland display or EGL).

## Usage

```bash
python imu_visualizer.py --list                      # detect serial ports
python imu_visualizer.py --port /dev/ttyACM0         # Linux (Pico USB CDC)
python imu_visualizer.py --port COM5                 # Windows
python imu_visualizer.py --port /dev/ttyACM0 --baud 115200
python imu_visualizer.py --sim                       # no hardware: synthetic tumbling
python imu_visualizer.py --sim --headless-test 3     # CI smoke test, exits 0
python imu_visualizer.py --sim --width 960 --height 720
```

Serial parameters: 115200 8N1 (baud is nominal for USB CDC), lines
terminated by `\n`.

### Controls

| Key        | Action                                  |
|------------|-----------------------------------------|
| Arrow keys | Orbit camera (azimuth / elevation)      |
| `r`        | Reset camera view                       |
| `t`        | Tare yaw (current heading becomes zero) |
| `ESC`      | Quit                                    |

## Attitude indicator (artificial horizon)

`attitude_indicator.py` is a second tool in the set: a classic aviation
attitude indicator (sky/ground ball, pitch ladder, bank scale, roll
pointer, fixed aircraft symbol) driven by the same stream. Same flags:

```bash
python attitude_indicator.py --list
python attitude_indicator.py --port /dev/ttyACM0
python attitude_indicator.py --sim                 # try it without hardware
python attitude_indicator.py --sim --headless-test 3
```

Controls: `ESC` quits. Display conventions: pitch up drops the horizon
below the fixed aircraft symbol; right roll slopes the horizon up to the
right and swings the roll pointer toward the right-hand bank marks
(ticks at 0/±10/±20/±30/±45/±60°). Pitch-ladder numbers ride the ball but
stay upright (EFIS style).

## Input data format (from firmware printf)

```
G:%6.2f %6.2f %6.2f dps | A:%5.3f %5.3f %5.3f g | Q:%6.3f %6.3f %6.3f %6.3f | RPY:%5.1f %5.1f %5.1f deg | HostUS:%llu
```

Example:

```
G: -0.20  -0.55   0.10 dps | A:0.034 0.034 1.031 g | Q: 0.962  0.035 -0.043 -0.266 | RPY:  5.2  -3.7 -31.1 deg | HostUS:36740365
```

Boot/diagnostic lines (`WHO_AM_I=...`, `CAL: ...`, `Gyro offsets: ...`)
are ignored by the parser.

## Conventions

- Quaternion order is **w, x, y, z** and expresses the **body frame
  relative to the NED world frame** (`v_ned = R(q) @ v_body`).
- NED: X = North (forward), Y = East (right), Z = Down.
- NED → OpenGL camera mapping (camera south of the body, looking north,
  slightly elevated): `E → +X_gl`, `Up(=-D) → +Y_gl`, `-N → +Z_gl`. The
  change of basis is a proper rotation (det = +1), so handedness is
  preserved; `ned_to_gl()` conjugates the body→NED rotation by it and
  returns a column-major 4×4 matrix for `glMultMatrixf`.
- World axes are drawn N=red (forward), E=green (right), D=blue (down);
  the aircraft model carries its own colored body axes so the rotation
  direction is unambiguous.
- Rendering interpolates between the last two quaternion samples with
  `slerp()` (sign-fixed: dot < 0 → negate, so the short path is taken);
  the sample period comes from the firmware `HostUS` timestamps.

## Files

```
imu_visualizer.py      # 3D rigid-body view: CLI, serial reader thread, GL renderer, HUD
attitude_indicator.py  # aviation artificial horizon (reuses the modules below)
serial_parser.py       # pure regex parser: parse_line(str) -> ImuSample|None
quat_math.py           # quat normalize/sign-fix/slerp/quat->mat3, NED->GL view matrix
sim_serial.py          # synthetic telemetry generator (also powers --sim mode)
test_parser.py         # unittest against real captured lines + quaternion math
test_attitude.py       # unittest: RPY extraction vs firmware, AI display geometry
requirements.txt
README.md
```

## Testing

```bash
python test_parser.py                        # parser + quaternion math (25 tests)
python test_attitude.py                      # attitude-indicator math (12 tests)
python imu_visualizer.py --sim --headless-test 3
python attitude_indicator.py --sim --headless-test 3
```

### Headless / CI note

`--headless-test N` creates a hidden pyglet window, runs the render loop
for N seconds offscreen, then exits 0. If the environment cannot provide
a GL context at all (no display/EGL, or missing GL system libraries such
as `libGLU`), the app **does not crash**: it prints a diagnostic and falls
back to a numpy-only pipeline check (sim telemetry → parser → slerp →
NED→GL matrix validation), still exiting 0. This is the case in the
sandbox this project was developed in — `libGLU` is not installed and
there is no display, so the GL window path itself could not be exercised
there; all non-GL logic (parser, quaternion math, slerp timing, reader
thread, tare, CLI) is covered by tests. On a normal desktop with a GPU,
`python imu_visualizer.py --sim` opens the window and animates.
