typedef struct {
    float h;      // altitude estimate <- this is the value used for height (never 100% certain so called estimate)
    float P;      // variance <- initial estimate
    float Q;      // process noise 
    float R;      // barometer noise from Data sheet, how wrong could the sensor be
} KF1D;

void kf_update(KF1D *kf, float z) {
    // Prediction step
    // P = P0 from first loop is a initial estimate & P_pred is the new estimate with added Process noise
    float P_pred = kf->P + kf->Q;
    
    // Update
    float y = z - kf->h; // This gives a difference between measured value and (prediction "last filtered measurement"), 
    // so if y = 0 kalman gain (K) multiplies by 0 giving 100% confidence. The reason Y is importantis it gives a values between the 
    // last measured height estimate to the current sensor measurement if the difference is big over cycles it corrects it self to a smaller error. 
    
    float S = P_pred + kf->R; // 
    float K = P_pred / S;     // these two make K = P_pred / (P_red + R) <- Kallman gain equation
    // the Kalman gain equation is a value between 0 and 1 which represent how confidence it is about the sensor reading given the noise of the sensor,
    // when the noise is very high the denominator dominates bringing the K value down. 
    
    kf->h = kf->h + K * y; // update step 
    kf->P = (1.0f - K) * P_pred; // 
}
