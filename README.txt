LSD_AVIONICS
Repository dedicated to designing algorithms used for avionics in embedded systems. 

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

1. Kalman filter

   - 1D single input linear Kallman filter (barometer example)


temp note to my self:
Linear algebra pracitce (to complete)

- week 1
- week 3
- week 5 
