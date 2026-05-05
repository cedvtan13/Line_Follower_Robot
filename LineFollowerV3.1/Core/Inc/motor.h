#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/**
 * @brief Initialize the motor driver (TIM2 PWM on PA0, PA1, PA2, PA3).
 */
void Motor_Init(void);

/**
 * @brief Set left motor speed and direction.
 * @param speed Speed from -1000 to 1000 (negative for reverse).
 */
void Motor_Left(int16_t speed);

/**
 * @brief Set right motor speed and direction.
 * @param speed Speed from -1000 to 1000 (negative for reverse).
 */
void Motor_Right(int16_t speed);

/**
 * @brief Stop both motors.
 */
void Motor_Stop(void);

#endif /* __MOTOR_H */
