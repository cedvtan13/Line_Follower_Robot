#include "pid.h"
#include <math.h>

void PID_Init(PID_Config *pid, float kp, float ki, float kd, int16_t base_speed, int16_t max_speed) {
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->base_speed = base_speed;
    pid->max_speed = max_speed;
    pid->last_error = 0;
    pid->integral = 0;
}

void PID_Compute(PID_Config *pid, int16_t current_position, int16_t *left_speed, int16_t *right_speed) {
    // Target position is 0 (centered on the line)
    float error = (float)current_position;

    // Proportional term
    float P = pid->Kp * error;

    // Derivative term
    float D = pid->Kd * (error - pid->last_error);
    pid->last_error = error;

    // Integral term
    pid->integral += error;
    // Basic clamp for integral
    if (pid->integral > 1000) pid->integral = 1000;
    if (pid->integral < -1000) pid->integral = -1000;
    
    float I = pid->Ki * pid->integral;
    
    float correction = P + I + D;

    // Arduino-style motor control (Inverted to match user's REVERSE PID logic):
    // If correction > 0, slow down LEFT wheel.
    // If correction < 0, slow down RIGHT wheel.
    
    int16_t l_spd, r_spd;
    
    if (correction > 0) {
        r_spd = pid->base_speed;
        l_spd = pid->base_speed - (int16_t)fabsf(correction);
    } else {
        l_spd = pid->base_speed;
        r_spd = pid->base_speed - (int16_t)fabsf(correction);
    }


    // Clamp speeds to -max_speed to max_speed
    if (l_spd > pid->max_speed) l_spd = pid->max_speed;
    if (l_spd < -pid->max_speed) l_spd = -pid->max_speed;
    if (r_spd > pid->max_speed) r_spd = pid->max_speed;
    if (r_spd < -pid->max_speed) r_spd = -pid->max_speed;
    
    *left_speed = l_spd;
    *right_speed = r_spd;
}

