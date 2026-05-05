#ifndef __IR_SENSOR_H
#define __IR_SENSOR_H

#include "main.h"

/**
 * @brief Initialize the IR sensor array driver.
 *        Configures PB1 as ADC input and PB2, PB8, PB9 as multiplexer select outputs.
 */
void IR_Init(void);

/**
 * @brief Select the multiplexer channel.
 * @param channel The channel to select (0-7).
 */
void IR_SelectChannel(uint8_t channel);

/**
 * @brief Read the ADC value of the currently selected channel.
 * @return 12-bit ADC value (0-4095).
 */
uint16_t IR_ReadADC(void);

/**
 * @brief Read all 8 IR sensors and store them in the provided array.
 * @param values Pointer to an array of 8 uint16_t to store the results.
 */
void IR_ReadAll(uint16_t *values);

/**
 * @brief Set the IR emitter brightness.
 * @param brightness Value from 0 to 255.
 */
void IR_SetBrightness(uint8_t brightness);

/**
 * @brief Compatibility API (no-op): calibration is not used.
 */
void IR_ResetCalibration(void);

/**
 * @brief Compatibility API (no-op): calibration is not used.
 */
void IR_Calibrate(void);

/**
 * @brief Calculate the line position using a weighted average of raw values.
 *        Uses raw ADC values directly (no calibration).
 * @return Line position from -3500 (far left) to 3500 (far right). 
 *         0 is centered. Returns -9999 if no line is detected.
 */
int16_t IR_GetLinePosition(void);

/**
 * @brief Check if a line is currently being detected by any sensor.
 * @return 1 if line detected, 0 if only white background is seen.
 */
uint8_t IR_IsLineDetected(void);

#endif /* __IR_SENSOR_H */
