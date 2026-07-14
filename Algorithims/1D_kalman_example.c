typedef struct {
    float h;      // altitude estimate
    float P;      // variance
    float Q;      // process noise
    float R;      // barometer noise
} KF1D;

void kf_update(KF1D *kf, float z) {
    // Predict
    float P_pred = kf->P + kf->Q;
    
    // Update
    float y = z - kf->h;
    float S = P_pred + kf->R;
    float K = P_pred / S;
    
    kf->h = kf->h + K * y;
    kf->P = (1.0f - K) * P_pred;
}
