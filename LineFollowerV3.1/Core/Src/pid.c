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
    // 82% old value, 18% new value. 
    // Slightly more damping to help eliminate the "very very little" overshoot.
    static float filtered_error = 0;
    filtered_error = (filtered_error * 0.82f) + (error * 0.18f);

    // Proportional
    float P = pid->Kp * filtered_error;

    // Derivative (On filtered signal)
    float D = pid->Kd * (filtered_error - pid->last_error);
    pid->last_error = filtered_error;

    float correction = P + D;

    // --- CENTER STABILIZER ---
    // Softened to 0.85x base speed to reduce "jittery" braking when near center.
    float abs_err = fabsf(filtered_error);
    if (abs_err < 1200.0f) {
        float max_c = pid->base_speed * 0.85f;
        if (correction > max_c) correction = max_c;
        if (correction < -max_c) correction = -max_c;
    }

    // --- DYNAMIC CORNER SNAP ---
    // Reduced factor from 4x to 3.5x for smoother high-speed corner entries.
    if (abs_err > 3500.0f) {
        float factor = 1.0f + ((abs_err - 3500.0f) / 3500.0f) * 2.5f; // Scale from 1x to 3.5x
        if (factor > 3.5f) factor = 3.5f;
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
