#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float last_error;
    float integral;
    int16_t max_speed;
    int16_t base_speed;
} PID_Config;

/* --- CONFIGURATION --- */
// Uncomment the line below to enable aggressive Micromouse-style 
// Active Pivot Braking (reversing the inner wheel on sharp turns).
// #define USE_ACTIVE_PIVOT

/**
 * @brief Initialize PID parameters.
 */
void PID_Init(PID_Config *pid, float kp, float ki, float kd, int16_t base_speed, int16_t max_speed);

/**
 * @brief Calculate motor speeds based on line position.
 * @param pid Pointer to PID configuration.
 * @param current_position Current line position (-100 to 100).
 * @param left_speed Pointer to store calculated left motor speed.
 * @param right_speed Pointer to store calculated right motor speed.
 */
void PID_Compute(PID_Config *pid, int16_t current_position, int16_t *left_speed, int16_t *right_speed);

#endif /* __PID_H */
