// Pure gyro quaternion integration (no accel, no drift compensation)

static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z
static void integrate_gyro(float gx_rad, float gy_rad, float gz_rad, float dt) {
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    // Quaternion derivative: q_dot = 0.5 * q ⊗ [0, ωx, ωy, ωz]
    float q_dot_w = 0.5f * (-qx*gx_rad - qy*gy_rad - qz*gz_rad);
    float q_dot_x = 0.5f * ( qw*gx_rad + qy*gz_rad - qz*gy_rad);
    float q_dot_y = 0.5f * ( qw*gy_rad - qx*gz_rad + qz*gx_rad);
    float q_dot_z = 0.5f * ( qw*gz_rad + qx*gy_rad - qy*gx_rad);
    // Integrate
    q[0] += q_dot_w * dt;
    q[1] += q_dot_x * dt;
    q[2] += q_dot_y * dt;
    q[3] += q_dot_z * dt;
    // Normalize
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0f) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    }
} // easy 
