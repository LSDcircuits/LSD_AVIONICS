#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"

#include "bmp388.h"
#include "icm42688p.h"

/*
 * Simple two-state vertical Kalman filter.
 *
 * State:
 *   x[0] = relative altitude, m
 *   x[1] = vertical velocity, m/s
 *
 * Core 1:
 *   Owns ICM and entire Kalman filter.
 *
 * Core 0:
 *   Owns BMP and sends altitude measurements to core 1.
 */

#define BMP_PERIOD_US             160000u  /* 6.25 Hz */
#define PRINT_PERIOD_US           200000u  /* 5 Hz */
#define NAV_PUBLISH_PERIOD_US      20000u  /* 50 Hz */

#define MAX_BARO_AGE_US           500000u
#define MAX_PREDICT_DT_S             0.020f

/* Starting values only. Tune later using stationary logs. */
#define BARO_STD_M                     0.20f
#define ACC_NOISE_DENSITY              0.12f

/* ---------------- Multicore communication ---------------- */

typedef struct {
    float z_m;
    uint64_t read_ts_us;
} baro_msg_t;

typedef struct {
    float z_m;
    float v_mps;
    float acc_up_mps2;
    float dt_s;

    uint32_t predictions;
    uint32_t baro_updates;
    uint32_t stale;

} nav_pub_t;

static queue_t baro_queue;
static mutex_t nav_mutex;
static nav_pub_t nav_pub;

static icm_t icm;
static pt_t bmp;

static volatile bool imu_ready = false;
static volatile bool imu_failed = false;

/* ---------------- Two-state Kalman filter ---------------- */

static struct {
    float x[2];
    float P[2][2];
    bool initialized;
    float dt_s;
    uint32_t predictions;
    uint32_t baro_updates;
    uint32_t stale;
} kf;

static void kf_reset(void) {
    // memset used to dump vals in Kf (&add, value, size of struct)
    memset(&kf, 0, sizeof(kf)); // sets all values in kf to 0.
    kf.initialized = false; // set handler to false
    // resets entire Kf in one operation
}
/* its equivalent of memset(&kf, 0, sizeof(kf));
kf.x[0] = 0.0f;
kf.x[1] = 0.0f;

kf.P[0][0] = 0.0f;
kf.P[0][1] = 0.0f;
kf.P[1][0] = 0.0f;
kf.P[1][1] = 0.0f;

kf.initialized = false;

kf.dt_s = 0.0f;
kf.predictions = 0;
kf.baro_updates = 0;
kf.stale = 0;
*/

static void kf_predict(float acc_up, float dt) {
    if (!kf.initialized ||
        !isfinite(acc_up) ||
        !(dt > 0.0f) ||
        dt > MAX_PREDICT_DT_S) {
        return;
    }

    float dt2 = dt * dt;
    float dt3 = dt2 * dt;

    //State prediction:
    kf.x[0] += kf.x[1] * dt + 0.5f * acc_up * dt2;

    kf.x[1] += acc_up * dt;


    // Covariance prediction:
    float p00 = kf.P[0][0];
    float p01 = kf.P[0][1];
    float p11 = kf.P[1][1];

    float sa2 = ACC_NOISE_DENSITY * ACC_NOISE_DENSITY;

    kf.P[0][0] = p00 + 2.0f * dt * p01 + dt2 * p11 + sa2 * dt3 / 3.0f;
    kf.P[0][1] = p01 + dt * p11 + sa2 * dt2 / 2.0f;
    kf.P[1][0] = kf.P[0][1];
    kf.P[1][1] = p11 + sa2 * dt;

    kf.dt_s = dt;
    kf.predictions++;
}

static void kf_update_baro(float z_baro) {
    if (!isfinite(z_baro)) {
        return;
    }

   // Initialize directly from the first valid BMP measurement.

    if (!kf.initialized) {
        memset(kf.P, 0, sizeof(kf.P));

        kf.x[0] = z_baro;
        kf.x[1] = 0.0f;

        kf.P[0][0] = BARO_STD_M * BARO_STD_M;

        kf.P[1][1] = 0.25f;

        kf.initialized = true;
        kf.baro_updates++;

        return;
    }

    
    // Barometer measurement:
    // Every finite BMP measurement is accepted.
    // There is deliberately no innovation gate.
    float R = BARO_STD_M * BARO_STD_M;
    float innovation =z_baro - kf.x[0];
    float S = kf.P[0][0] + R;

    if (!(S > 0.0f) || !isfinite(S)) {
    // this should never happen normally.
    // Restart rather than allowing invalid covariance to propagate.
        kf_reset();
        return;
    }

    float K0 = kf.P[0][0] / S;
    float K1 = kf.P[1][0] / S;

    float p00 = kf.P[0][0];
    float p01 = kf.P[0][1];
    float p11 = kf.P[1][1];

    /* State update. */
    kf.x[0] += K0 * innovation;
    kf.x[1] += K1 * innovation;


    //Joseph covariance update for H = [1, 0].

    kf.P[0][0] = (1.0f - K0) * (1.0f - K0) * p00 + K0 * K0 * R;
    kf.P[0][1] = (1.0f - K0) * (p01 - K1 * p00) + K0 * K1 * R;
    kf.P[1][0] = kf.P[0][1];
    kf.P[1][1] = p11 - K1 * p01 + K1 * K1 * R;

    kf.baro_updates++;
}

static void publish_nav(void) {
    mutex_enter_blocking(&nav_mutex);

    nav_pub.z_m = kf.x[0];
    nav_pub.v_mps = kf.x[1];
    nav_pub.acc_up_mps2 = icm.acc_up;
    nav_pub.dt_s = kf.dt_s;
    nav_pub.predictions = kf.predictions;
    nav_pub.baro_updates = kf.baro_updates;
    nav_pub.stale = kf.stale;

    mutex_exit(&nav_mutex);
}

/* ---------------- Core 1: ICM and Kalman filter ---------------- */

void attitude_core(void) {

    if (!imu_init()) {
        imu_failed = true;
        while (true) {
            tight_loop_contents();
        }
    }

    printf("Keep IMU stationary for calibration...\n");

    if (!calibrate_imu(2000)) {
        imu_failed = true;
        while (true) {
            tight_loop_contents();
        }
    }

    kf_reset();
    icm_t previous_icm;
    memset(&previous_icm, 0, sizeof(previous_icm));
    bool have_previous = false;
    uint64_t previous_ts = 0;
    uint64_t next_publish = time_us_64();

    imu_ready = true;

    while (true) {

        // Consume every available BMP measurement once.

        baro_msg_t msg;

        while (queue_try_remove(&baro_queue, &msg)) {
            uint64_t now = time_us_64();

            if (now >= msg.read_ts_us && now - msg.read_ts_us <= MAX_BARO_AGE_US) {

                kf_update_baro(msg.z_m);
            } else {
                kf.stale++;
            }
        }

        get_imu(&icm);


        //get_imu() leaves its output unchanged if no FIFO packet
        //was available. Do not integrate stale acc_up repeatedly.

        bool fresh =
            !have_previous ||
            memcmp(&icm, &previous_icm, sizeof(icm)) != 0;
        if (fresh) {
            uint64_t now = time_us_64();
            if (have_previous) {
                float dt = (float)(now - previous_ts) * 1.0e-6f;
                kf_predict(icm.acc_up, dt);
            }
            previous_ts = now;
            previous_icm = icm;
            have_previous = true;
        }
        uint64_t now = time_us_64();
        if ((int64_t)(now - next_publish) >= 0) {
            next_publish = now + NAV_PUBLISH_PERIOD_US;

            publish_nav();
        }
    }
}

/* ---------------- Core 0: BMP acquisition and output ---------------- */

int main(void) {
    stdio_init_all();
    queue_init(&baro_queue, sizeof(baro_msg_t), 8);
    mutex_init(&nav_mutex);
    multicore_launch_core1(attitude_core);

    if (!bmp_init()) {
        printf("BMP388 initialization failed.\n");
        while (true) {
            tight_loop_contents();
        }
    }

     //Allow the BMP configuration and conversions to stabilize.

    sleep_ms(1000);


     //Establish pressure reference while stationary.
     //Twenty samples at 160 ms each takes about 3.2 seconds.

    get_p0(&bmp, 20);

    printf("BMP reference p0 = %.3f Pa\n",(double)bmp.p0);

    while (!imu_ready) {
        if (imu_failed) {
            printf("IMU initialization/calibration failed.\n");
            while (true) {
                tight_loop_contents();
            }
        }
        sleep_ms(10);
    }

    uint32_t dropped = 0;
    uint64_t next_bmp = time_us_64();
    uint64_t next_print = next_bmp;

    while (true) {
        uint64_t now = time_us_64();

        /*
         * Read BMP independently at 6.25 Hz.
         */
        if ((int64_t)(now - next_bmp) >= 0) {
            get_bmp(&bmp);
            baro_msg_t msg = { .z_m = (float)bmp.bmp_z, .read_ts_us = time_us_64()};
            if (!queue_try_add(&baro_queue, &msg)) {
                dropped++;
            }
            next_bmp = now + BMP_PERIOD_US;
        }

        /*
         * Print fused output and raw pressure diagnostics.
         */
        now = time_us_64();

        if ((int64_t)(now - next_print) >= 0) {
            next_print = now + PRINT_PERIOD_US;

            nav_pub_t nav;

            mutex_enter_blocking(&nav_mutex);
            nav = nav_pub;
            mutex_exit(&nav_mutex);

            printf(
                "BARO:%8.3f m | "
                "FUSED:%8.3f m  "
                "V:%7.3f m/s | "
                "aIMU:%7.3f  "
                "dt:%6.4f | "
                "BMP updates/stale/drop:%lu/%lu/%lu | "
                "P:%.1f P0:%.1f Pa\n",
                (double)bmp.bmp_z,
                (double)nav.z_m,
                (double)nav.v_mps,
                (double)nav.acc_up_mps2,
                (double)nav.dt_s,
                (unsigned long)nav.baro_updates,
                (unsigned long)nav.stale,
                (unsigned long)dropped,
                (double)bmp.press,
                (double)bmp.p0);
        }

        sleep_ms(1);
    }
    return 0;
}
