#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "pico/util/queue.h"

typedef struct {
    float z_m;
    uint64_t read_ts_us;
} baro_msg_t;

static queue_t baro_queue;       // Queue control + storage information
static mutex_t nav_mutex;        // Protects nav_pub
static baro_msg_t nav_pub;       // Shared memory

void core1_main(void) {
    while (true) {
        baro_msg_t msg;          // Local memory on core 1

        // Queue storage -> core 1's local msg
        if (queue_try_remove(&baro_queue, &msg)) {

            mutex_enter_blocking(&nav_mutex);

            // Core 1's local msg -> shared memory
            nav_pub = msg;

            mutex_exit(&nav_mutex);
        }

        sleep_ms(1);
    }
}

int main(void) {
    stdio_init_all();

    // Queue holds 8 messages, each sizeof(baro_msg_t) bytes
    queue_init(&baro_queue, sizeof(baro_msg_t), 8);
    mutex_init(&nav_mutex);

    multicore_launch_core1(core1_main);

    float altitude = 10.0f;

    while (true) {
        baro_msg_t msg = {       // Local memory on core 0
            .z_m = altitude,
            .read_ts_us = time_us_64()
        };

        // Core 0's local msg -> queue storage (a copy is made)
        if (queue_try_add(&baro_queue, &msg)) {
            printf("Core 0 put %.2f into queue\n", msg.z_m);
        } else {
            printf("Queue full: message dropped\n");
        }

        sleep_ms(50);            // Give core 1 time to receive it

        baro_msg_t copy;         // Another local variable on core 0

        mutex_enter_blocking(&nav_mutex);

        // Shared memory -> core 0's local copy
        copy = nav_pub;

        mutex_exit(&nav_mutex);

        printf("Core 0 read %.2f from shared memory\n\n", copy.z_m);

        altitude += 0.5f;
        sleep_ms(160);
    }
}
