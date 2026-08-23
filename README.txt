LSD_AVIONICS
Repository dedicated to designing algorithms & testing used for avionics in embedded systems for fumca.

Main process(to be optimized if needed)

IMU (gyro, accel, mag)
    │
    ├──→ Madgwick ──→ Quaternion ──→ Attitude (roll, pitch, yaw)
    │                      │
    │                      └──→ Rotation matrix R
    │                              │
    │                              └──→ Rotate accel to world frame
    │                                      │
    │                                      └──→ Subtract gravity
    │                                              │
    │                                              └──→ Linear acceleration
    │                                                      │
    └──→ Kalman prediction (integrate accel for velocity/position)
            │
            └──→ GPS arrives ──→ Kalman update (correct position/velocity)

The PIPELINES made in PICO RP2350 used to test sensors, the code structure is kept using the higher level functions from the pico-sdk so the entire bus fabric is handled by the function provided by the vendor sdk. The pipelines are kept at register level for more flexible compatability with other controllers such as the STM32's or NXP LPC series. the pipelines are a small part of the project which need to be well defined for predictable uses when filtering nouse or integrating data into usable values.

Algorithms, this section is used to describe different algorithms used for filtering any data but also for planning the architecture since its heavily time based and the rp2350 has 2 arm cores which is what will be used alternatively it can also be split to the risk 5 cores it has but then would require different architecture planning affecting how algorithms are applied. 





