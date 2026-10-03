// this file isnt a fucntional code file, itsa just expresinf similar algorithms to compare, generaly in code some algorithms have memory & others have hard set values
// some use High pass other use Concrete numbers & parabolas, i will be going over both methods on a variable gain Mahony filter.

// Variable range from Hardcoded values
    float norm;
    float gain;
    float n = 3;
    float K = 1;
    float ex = 0.0f, ey = 0.0f, ez = 0.0f;
    // Normalize accelerometer (only correct attitude when not in freefall)
    norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    gain = -((n*1 - n*norm)*(n*1 - n*norm)) + K;
    gain = fmaxf(0.0f, gain); // clamped
    float MAHONY_KP =  MAHONY_KP_0 * gain;
    float MAHONY_KI =  MAHONY_KI_0 * gain;

// Frequency based

// --- persistent state (file scope, next to q[] and gyro_bias_mahony[]) ---
static float s_lp = 1.0f;

// --- inside mahony_update, after norm = sqrtf(...) ---
norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

s_lp += 0.02f * (norm - s_lp);                    // smooth the DECISION variable
float d = s_lp - 1.0f;
float gain = fmaxf(0.0f, 1.0f - (d/0.333f)*(d/0.333f));  // your parabola, fed s_lp

float kp_eff = MAHONY_KP_0 * gain;
float ki_eff = MAHONY_KI_0 * gain;

if (norm > 0.1f) {                                // gate on RAW norm (degenerate-vector guard only)
    ... // normalize with raw norm, error, KI with ki_eff, correct with kp_eff + bias
}
