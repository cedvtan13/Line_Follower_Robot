#include "motor.h"
#include <stdlib.h>

/*
 * Motor Driver: DRV8871
 * Left Motor:  PA0 (IN1), PA1 (IN2) -> TIM2 CH1, TIM2 CH2
 * Right Motor: PA2 (IN1), PA3 (IN2) -> TIM2 CH3, TIM2 CH4
 */

void Motor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 1. Enable Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 2. Configure PA0, PA1, PA2, PA3 as Alternate Function (TIM2) */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 3. Configure TIM2 for PWM */
    TIM2->PSC = 0;      // No prescaler
    TIM2->ARR = 2399;   // 2400 steps for 20kHz

    /* Configure Channels 1, 2, 3, 4 as PWM mode 1 */
    TIM2->CCMR1 = (6U << 4) | (6U << 12); // CH1, CH2
    TIM2->CCMR2 = (6U << 4) | (6U << 12); // CH3, CH4
    
    /* Enable Preload */
    TIM2->CCMR1 |= (1U << 3) | (1U << 11);
    TIM2->CCMR2 |= (1U << 3) | (1U << 11);

    /* Enable Preload and Output for all channels */
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    /* Generate an update event to reload PSC and ARR */
    TIM2->EGR = TIM_EGR_UG;

    /* Start Timer */
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Motor_Left(int16_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    // SLOW DECAY (BRAKING) LOGIC:
    // To move FORWARD: Pin1 = 2399 (HIGH), Pin2 = 2399 - duty (PWM)
    // This shorts the motor to GND during the "off" cycle, providing active braking.
    if (speed >= 0) {
        uint32_t duty = (uint32_t)speed * 2399 / 1000;
        TIM2->CCR1 = 2399;
        TIM2->CCR2 = 2399 - duty;
    } else {
        // BACKWARD
        uint32_t duty = (uint32_t)(-speed) * 2399 / 1000;
        TIM2->CCR1 = 2399 - duty;
        TIM2->CCR2 = 2399;
    }
}

void Motor_Right(int16_t speed) {
    if (speed > 1000) speed = 1000;
    if (speed < -1000) speed = -1000;

    if (speed >= 0) {
        // FORWARD
        uint32_t duty = (uint32_t)speed * 2399 / 1000;
        TIM2->CCR3 = 2399;
        TIM2->CCR4 = 2399 - duty;
    } else {
        // BACKWARD
        uint32_t duty = (uint32_t)(-speed) * 2399 / 1000;
        TIM2->CCR3 = 2399 - duty;
        TIM2->CCR4 = 2399;
    }
}

void Motor_Stop(void) {
    // ACTIVE BRAKE: DRV8871 brakes when both IN1 and IN2 are HIGH
    TIM2->CCR1 = 2399;
    TIM2->CCR2 = 2399;
    TIM2->CCR3 = 2399;
    TIM2->CCR4 = 2399;
}
