#include "pid.h"
#include <math.h>

void PID_Init(PID_Config *pid, float kp, float ki, float kd, int16_t base_speed, int16_t max_speed) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->base_speed = base_speed;
    pid->max_speed = max_speed;
    pid->integral = 0;
    pid->last_error = 0;
}

void PID_Compute(PID_Config *pid, int16_t line_position, int16_t *left_speed, int16_t *right_speed) {
    float error = (float)line_position;

    // --- BALANCED HIGH-SPEED FILTER ---
    // 80% old value, 20% new value. 
    // Faster reaction to catch 750-speed curves before they overshoot.
    static float filtered_error = 0;
    filtered_error = (filtered_error * 0.80f) + (error * 0.20f);

    // Proportional
    float P = pid->Kp * filtered_error;

    // Derivative (On filtered signal)
    float D = pid->Kd * (filtered_error - pid->last_error);
    pid->last_error = filtered_error;

    float correction = P + D;

    // --- CENTER STABILIZER ---
    // Lowered to 1200 to allow aggressive differential force to start sooner.
    float abs_err = fabsf(filtered_error);
    if (abs_err < 1200.0f) {
        float max_c = pid->base_speed * 0.90f;
        if (correction > max_c) correction = max_c;
        if (correction < -max_c) correction = -max_c;
    }

    // --- DYNAMIC CORNER SNAP ---
    // Faster kick (3500) and more power (4x) for extreme speed surviving.
    if (abs_err > 3500.0f) {
        float factor = 1.0f + ((abs_err - 3500.0f) / 3500.0f) * 3.0f; // Scale from 1x to 4x
        if (factor > 4.0f) factor = 4.0f;
        correction *= factor;
    }
    
    // Differential Steering (Corrected for your verified sensor/motor alignment)
    int16_t l_out = pid->base_speed - (int16_t)correction;
    int16_t r_out = pid->base_speed + (int16_t)correction;
    
    // Clamp to limits
    if (l_out > pid->max_speed) l_out = pid->max_speed;
    if (l_out < -pid->max_speed) l_out = -pid->max_speed;
    if (r_out > pid->max_speed) r_out = pid->max_speed;
    if (r_out < -pid->max_speed) r_out = -pid->max_speed;
    
    *left_speed = l_out;
    *right_speed = r_out;
}
