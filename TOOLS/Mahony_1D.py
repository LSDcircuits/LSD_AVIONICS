"""Minimal 1D Mahony filter. Run with F5 in Spyder."""
import numpy as np
import matplotlib.pyplot as plt

DT = 0.001          # 1 kHz
KP = 0.5            # proportional gain
KI = 0.3            # integral gain

y = 0.0             # estimated angle [rad]
bias = 0.0          # gyro bias estimate

def mahony_1d(ref, gyro):
    """One filter step. ref = accel angle [rad], gyro = gyro rate [rad/s].
    Returns the estimated angle."""
    global y, bias                      # 'global' = use the y,bias above (like C statics)
    e = ref - y                         # error
    bias += KI * e * DT                 # integrate error (Ki branch)
    y += (gyro + KP * e + bias) * DT    # integrate corrected rate (1/s)
    return y

# --- demo: true angle steps to 0.35 rad (20 deg), gyro has a 0.1 rad/s bias ---
ref = np.full(20000, 0.35)              # accel says "20 deg" the whole time
gyro = 0.1 + np.zeros(20000)            # gyro: 0.1 rad/s constant bias, no motion
est = np.zeros(20000)
for k in range(20000):
    est[k] = mahony_1d(ref[k], gyro[k])

t = np.arange(20000) * DT
plt.plot(t, est * 180/np.pi, label="estimate")
plt.axhline(20, color="k", ls="--", label="true angle (20 deg)")
plt.xlabel("time [s]"); plt.ylabel("angle [deg]")
plt.legend(); plt.grid(True); plt.show()
print("final estimate: %.2f deg (true: 20 deg)" % (est[-1] * 180/np.pi))
