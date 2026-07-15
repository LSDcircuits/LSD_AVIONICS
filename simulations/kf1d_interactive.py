#!/usr/bin/env python3
"""
Interactive 1D Kalman Filter Simulation
Displays: state estimate (h), Kalman gain (K), and uncertainty (P)
Sliders control Q, R, initial P, and measurement noise level.
Reset button restores defaults.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# ─── CONFIG ──────────────────────────────────────────
N_STEPS = 200
TRUE_ALTITUDE = 100.0          # meters (constant for this demo)
DEFAULT_Q = 0.5                # process noise
DEFAULT_R = 10.0               # measurement noise (barometer)
DEFAULT_P0 = 50.0            # initial uncertainty
DEFAULT_NOISE_AMP = 8.0        # how noisy the synthetic sensor is
SEED = 42

# ─── KALMAN FILTER STRUCT & UPDATE ───────────────────
class KF1D:
    def __init__(self, h=0.0, P=1.0, Q=0.1, R=1.0):
        self.h = h
        self.P = P
        self.Q = Q
        self.R = R
        # history arrays
        self.h_hist = []
        self.P_hist = []
        self.K_hist = []

    def reset(self, h, P, Q, R):
        self.h = h
        self.P = P
        self.Q = Q
        self.R = R
        self.h_hist.clear()
        self.P_hist.clear()
        self.K_hist.clear()

    def update(self, z):
        # Prediction
        P_pred = self.P + self.Q
        # Innovation
        y = z - self.h
        S = P_pred + self.R
        K = P_pred / S
        # Update
        self.h = self.h + K * y
        self.P = (1.0 - K) * P_pred
        # Record
        self.h_hist.append(self.h)
        self.P_hist.append(self.P)
        self.K_hist.append(K)

# ─── GENERATE SYNTHETIC SENSOR DATA ──────────────────
def generate_measurements(n, true_val, noise_amp, seed=SEED):
    rng = np.random.default_rng(seed)
    return true_val + rng.normal(0, noise_amp, n)

# ─── SETUP FIGURE & AXES ─────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.subplots_adjust(left=0.1, bottom=0.35)

ax_h, ax_k, ax_p = axes
ax_h.set_title("State Estimate  $h$  (altitude)")
ax_h.set_ylabel("Altitude [m]")
ax_h.axhline(TRUE_ALTITUDE, color='green', linestyle='--', alpha=0.6, label='True altitude')

ax_k.set_title("Kalman Gain  $K$")
ax_k.set_ylabel("Gain [0–1]")
ax_k.set_ylim(-0.05, 1.05)

ax_p.set_title("Uncertainty  $P$")
ax_p.set_ylabel("Variance [m²]")
ax_p.set_xlabel("Time Step")

# Initial empty plots
line_z, = ax_h.plot([], [], 'o', color='gray', alpha=0.3, markersize=3, label='Measurement $z$')
line_h, = ax_h.plot([], [], '-', color='blue', lw=2, label='Estimate $h$')
line_k, = ax_k.plot([], [], '-', color='red', lw=2)
line_p, = ax_p.plot([], [], '-', color='purple', lw=2)
ax_h.legend(loc='upper right')

# ─── SLIDER AXES ─────────────────────────────────────
ax_q     = fig.add_axes([0.15, 0.26, 0.65, 0.03])
ax_r     = fig.add_axes([0.15, 0.22, 0.65, 0.03])
ax_p0    = fig.add_axes([0.15, 0.18, 0.65, 0.03])
ax_noise = fig.add_axes([0.15, 0.14, 0.65, 0.03])

slider_q     = Slider(ax_q,     'Q (process noise)',   0.0, 20.0, valinit=DEFAULT_Q, valstep=0.1)
slider_r     = Slider(ax_r,     'R (sensor noise)',    0.1, 100.0, valinit=DEFAULT_R, valstep=0.5)
slider_p0    = Slider(ax_p0,    'P₀ (init uncert)',    0.0, 200.0, valinit=DEFAULT_P0, valstep=1.0)
slider_noise = Slider(ax_noise, 'Synth noise amp',     0.0, 30.0, valinit=DEFAULT_NOISE_AMP, valstep=0.5)

# ─── RESET BUTTON ────────────────────────────────────
ax_reset = fig.add_axes([0.8, 0.08, 0.1, 0.04])
btn_reset = Button(ax_reset, 'Reset', color='lightgoldenrodyellow', hovercolor='0.975')

# ─── UPDATE FUNCTION ───────────────────────────────────
def update(val=None):
    Q = slider_q.val
    R = slider_r.val
    P0 = slider_p0.val
    noise_amp = slider_noise.val

    # Re-run filter with current slider values
    kf = KF1D(h=0.0, P=P0, Q=Q, R=R)
    zs = generate_measurements(N_STEPS, TRUE_ALTITUDE, noise_amp)
    for z in zs:
        kf.update(z)

    t = np.arange(N_STEPS)
    line_z.set_data(t, zs)
    line_h.set_data(t, kf.h_hist)
    line_k.set_data(t, kf.K_hist)
    line_p.set_data(t, kf.P_hist)

    # Dynamic y-limits for h and P
    ax_h.set_xlim(0, N_STEPS)
    ax_h.set_ylim(min(zs.min(), TRUE_ALTITUDE - 10), max(zs.max(), TRUE_ALTITUDE + 10))
    ax_p.set_ylim(0, max(kf.P_hist) * 1.2 if kf.P_hist else 10)

    fig.canvas.draw_idle()

# ─── RESET CALLBACK ──────────────────────────────────
def reset(event):
    slider_q.reset()
    slider_r.reset()
    slider_p0.reset()
    slider_noise.reset()
    update()

btn_reset.on_clicked(reset)

# Connect sliders
slider_q.on_changed(update)
slider_r.on_changed(update)
slider_p0.on_changed(update)
slider_noise.on_changed(update)

# Initial draw
update()

plt.show()
