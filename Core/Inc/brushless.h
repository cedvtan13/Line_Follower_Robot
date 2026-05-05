#ifndef __BRUSHLESS_H
#define __BRUSHLESS_H

#include "main.h"

#define DSHOT_MAX_VALUE 2000

/**
 * @brief Initialize DShot300 for brushless motors using TIM1 DMA.
 */
void Brushless_Init(void);

/**
 * @brief Set speed for both brushless motors (0-2000).
 */
void Brushless_SetSpeed(uint16_t motor1, uint16_t motor2);

#endif /* __BRUSHLESS_H */
