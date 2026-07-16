#include <math.h>

// this one will take a hefty ammoun of time. 

typedef struct {
    float h;        // height [m]
    float v;        // velocity [m/s]
    float b;        // accel bias [m/s^2]

    // Covariance matrix P (symmetric, 3x3)
    // [ p00  p01  p02 ]
    // [ p01  p11  p12 ]
    // [ p02  p12  p22 ]
    float p00, p01, p02;
    float p11, p12;
    float p22;

    // Process noise Q (only diagonal needed here)
    float q_h, q_v, q_b;

    // Measurement noise
    float R;
    float g;
} KF3;

/* ------------------------------------------------------------------ */
void kf_init(KF3 *kf, float R_baro, float gravity)
{
    kf->h = 0.0f;
    kf->v = 0.0f;
    kf->b = 0.0f;

    // Start with "I have no idea"
    kf->p00 = 100.0f;  kf->p01 = 0.0f;   kf->p02 = 0.0f;
    kf->p11 = 100.0f;  kf->p12 = 0.0f;
    kf->p22 = 10.0f;

    // Tune these three to your expected motion
    kf->q_h = 0.0001f;   // m^2 per step
    kf->q_v = 0.001f;    // (m/s)^2 per step
    kf->q_b = 0.00001f;  // (m/s^2)^2 per step

    kf->R = R_baro;
    kf->g = gravity;
}

/* ------------------------------------------------------------------ */
/* PREDICT: I move forward in time using the accelerometer            */
void kf_predict(KF3 *kf, float a_meas, float dt)
{
    // 1. What is the true acceleration? (measured - gravity - bias)
    float a_true = a_meas - kf->g - kf->b;

    // 2. Move my state estimate forward
    kf->h = kf->h + kf->v * dt + 0.5f * a_true * dt * dt;
    kf->v = kf->v + a_true * dt;
    // bias stays the same (random walk)

    // 3. Grow my uncertainty because I moved in time
    // P_pred = P + Q  (simple, because we assume F ≈ I for the covariance)
    kf->p00 += kf->q_h;
    kf->p11 += kf->q_v;
    kf->p22 += kf->q_b;
}

/* ------------------------------------------------------------------ */
/* UPDATE: I got a barometer reading. Fuse it in.                     */
void kf_update(KF3 *kf, float z_baro)
{
    // 1. How surprised am I? (sensor vs prediction)
    float y = z_baro - kf->h;

    // 2. Total uncertainty of that surprise
    // S = H*P*H^T + R, and H = [1, 0, 0], so S = p00 + R
    float S = kf->p00 + kf->R;

    // 3. Kalman gain: how much do I trust this surprise?
    // K = P * H^T / S  =  [p00; p01; p02] / S
    float K_h = kf->p00 / S;
    float K_v = kf->p01 / S;
    float K_b = kf->p02 / S;

    // 4. Correct my state estimate
    kf->h += K_h * y;
    kf->v += K_v * y;
    kf->b += K_b * y;

    // 5. Shrink my uncertainty because I learned something
    // P_new = (I - K*H) * P
    // K*H only touches the first row, so we subtract K * [p00 p01 p02]
    float row0_0 = kf->p00;
    float row0_1 = kf->p01;
    float row0_2 = kf->p02;

    kf->p00 -= K_h * row0_0;
    kf->p01 -= K_h * row0_1;
    kf->p02 -= K_h * row0_2;

    kf->p01 -= K_v * row0_0;   // p10
    kf->p11 -= K_v * row0_1;
    kf->p12 -= K_v * row0_2;

    kf->p02 -= K_b * row0_0;   // p20
    kf->p12 -= K_b * row0_1;
    kf->p22 -= K_b * row0_2;

    // Force symmetry (cheap fix for numerical drift)
    // p01 = p10, p02 = p20, p12 = p21
    // (already handled above since we wrote to both sides)
}

/* ------------------------------------------------------------------ */
/* EXAMPLE LOOP                                                       */
/*
#define DT 0.1f

int main(void)
{
    KF3 kf;
    kf_init(&kf, 0.01f, 9.81f);   // R = 0.01 m^2, g = 9.81

    while (1) {
        float a_meas = read_accel_z();      // m/s^2
        float z_baro = read_baro_meters();  // m

        kf_predict(&kf, a_meas, DT);
        kf_update(&kf, z_baro);

        // kf.h is your best altitude
        // kf.v is your best vertical speed
        // kf.b is the estimated bias
    }
}
*/
