#include "motor.h"
#include "tim.h"
#include <stdlib.h>

/*
 * Motor Driver: DRV8871 (Differential Drive)
 * 
 * Hardware Mapping (from User):
 * Left Motor:  OUT1(+) OUT2(-) -> PA0(CH1), PA1(CH2)
 * Right Motor: OUT1(-) OUT2(+) -> PA2(CH3), PA3(CH4)
 * 
 * Logic to move FORWARD:
 * Left:  IN1=PWM, IN2=LOW  (OUT1=H, OUT2=L)
 * Right: IN1=LOW, IN2=PWM  (OUT1=L, OUT2=H)
 */

void Motor_Init(void) {
    // Timer 2 ARR must be 2399 for 20kHz PWM
    __HAL_TIM_SET_AUTORELOAD(&htim2, 2399);
    
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    
    Motor_Stop();
}

void Motor_Left(int16_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    uint32_t duty = (uint32_t)abs(speed) * 2399 / 1000;

    if (speed >= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty);
    }
}

void Motor_Right(int16_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;
    uint32_t duty = (uint32_t)abs(speed) * 2399 / 1000;

    if (speed >= 0) {
        // Now Forward: IN1=PWM, IN2=LOW (or whatever matches your wires to go forward)
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    } else {
        // Now Backward
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty);
    }
}

void Motor_Stop(void) {
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}
