typedef struct {
    float Q;      // process noise <- uncertanty of prediction model.
                  // where to get Q from? from 2 samples how much can the altitude realistically change?
                  // If a aicraft has 1 m/s vertical speed, rate is 10HZ = 0.1s max expected vertical measurment is 0.1m or 10cm.
                  // since its a prediction we can call it the standard deviation of the current model, Q must be in variance so it must be squared 
                  // so Q = (0.1)^2 = 0.01 m^2  (for the prediction step)

    float R;      // Barometer noise <- uncertanty of measurement. 
                  // This is used for the update step, its important for the update step since it controlls the gain used for the next prediciton
                  // this directly effect the value of K and the updated P 

    float P;      // current uncertanty <- based on Prediction and Kalman gain.

    float h;      // altitude estimate <- this is the value used for height (never 100% certain so called estimate)

} KF1D;

void kf_update(KF1D *kf, float z) {
    // Prediction step
    // P_pred is the predicted covariance (how uncertain you are before looking at the sensor).
    float P_pred = kf->P + kf->Q;
    
    // Update
    float y = z - kf->h; // This gives a difference between measured value and (prediction "last filtered measurement"), 
    // so if y = 0 kalman gain (K) multiplies by 0 giving 100% confidence. The reason Y is important is it gives a values between the 
    // last measured height estimate to the current sensor measurement if the difference is big over cycles it corrects it self to a smaller error. 
    
    float S = P_pred + kf->R; // 
    float K = P_pred / S;     // these two make K = P_pred / (P_red + R) <- Kallman gain equation
    // the Kalman gain equation is a value between 0 and 1 which represent how confidence it is about the sensor reading given the noise of the sensor,
    // when the noise is very high the denominator dominates bringing the K value down. 
    // A importnat disticntion of K is that it represents the confidence of the sensor relative to the predicttion. 
    
    kf->h = kf->h + K * y; // update step 
    kf->P = (1.0f - K) * P_pred; // 
}
