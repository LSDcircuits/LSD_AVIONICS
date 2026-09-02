LSD_AVIONICS
Repository dedicated to designing algorithms & testing used for avionics in embedded systems for fumca. Main goal is to create a geolocator which can be used to send telemetry ack to the home base using off shelf sensors and embedded solutions and compare it to Satlocs high presicion avionics. 

Main process(to be optimized if needed)

IMU (gyro, accel, mag)
    │
    ├──→ Mahony ──→ Quaternion ──→ Attitude (roll, pitch, yaw)
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

Simulations, this section is used to test different algorithms and performance issues in pipelines. The simulations are made in python (using AI) this gives me slightly more freedom than using Matlab and has the possibility of providing more visual aids and speeding up the process. 

Drafts, this section consist of documentation used for the code ive constructed for the RP2350 but also important explanations of the maths used to derive specific outputs such as quaternions and how they're used for rotation but also how much process noise is created from this method which is very important for prediction steps on kallman filters.

This project is slightly over a month old and is in its initial phase. 




