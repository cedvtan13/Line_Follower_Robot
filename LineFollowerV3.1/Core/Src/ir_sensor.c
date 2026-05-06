#include "ir_sensor.h"
#include "tim.h"

/* 
 * DYNAMIC SENSOR CALIBRATION
 * Stores the minimum (white) and maximum (black) values seen during the sweep.
 */
static uint16_t sensor_min[8] = { 4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095 };
static uint16_t sensor_max[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
static uint16_t thresholds[8] = { 3100, 3100, 3100, 3100, 3100, 3100, 3100, 3100 };

#define IR_POSITION_SCALE_NUM   15
#define IR_POSITION_SCALE_DEN   10

static uint16_t last_raw_values[8];

void IR_StartCalibration(void) {
    for (int i = 0; i < 8; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }
}

void IR_UpdateCalibration(void) {
    uint16_t current_vals[8];
    IR_ReadAll(current_vals);
    for (int i = 0; i < 8; i++) {
        if (current_vals[i] < sensor_min[i]) sensor_min[i] = current_vals[i];
        if (current_vals[i] > sensor_max[i]) sensor_max[i] = current_vals[i];
    }
}

void IR_FinishCalibration(void) {
    for (int i = 0; i < 8; i++) {
        // If range is too small, fallback to default threshold
        if (sensor_max[i] - sensor_min[i] > 500) {
            thresholds[i] = sensor_min[i] + (sensor_max[i] - sensor_min[i]) / 2;
        } else {
            thresholds[i] = 3100;
        }
    }
}

void IR_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    IR_SetBrightness(255);
    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC1->CR1 = 0; 
    ADC1->CR2 = ADC_CR2_ADON; 
    ADC1->SMPR2 |= (7U << 27);
}

void IR_SelectChannel(uint8_t channel) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for(volatile int i=0; i<800; i++); 
}

uint16_t IR_ReadADC(void) {
    ADC1->SQR3 = 9;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

void IR_SetBrightness(uint8_t brightness) {
    uint32_t duty = (255 - brightness) * 2399 / 255;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}

void IR_ReadAll(uint16_t *values) {
    for (uint8_t i = 0; i < 8; i++) {
        IR_SelectChannel(i);
        values[i] = IR_ReadADC();
        last_raw_values[i] = values[i];
    }
}

int16_t IR_GetLinePosition(void) {
    unsigned long avg = 0;
    unsigned long sum = 0;
    uint16_t max_val = 0;

    for (uint8_t i = 0; i < 8; i++) {
        uint16_t raw = last_raw_values[i];
        
        // Subtract calibrated min (white) to get true signal strength
        int32_t signal = (int32_t)raw - (int32_t)sensor_min[i];
        if (signal < 0) signal = 0;
        
        // Only use sensor if it's significantly darker than its calibrated floor
        if (raw > thresholds[i]) {
            avg += (unsigned long)signal * (i * 1000U);
            sum += signal;
            if (raw > max_val) max_val = raw;
        }
    }

    if (sum == 0) return -9999;

    int32_t raw_pos = (int32_t)(avg / sum);
    int32_t mapped_pos = raw_pos - 3500;
    int32_t scaled_pos = (mapped_pos * IR_POSITION_SCALE_NUM) / IR_POSITION_SCALE_DEN;
    
    if (scaled_pos > 7000) scaled_pos = 7000;
    if (scaled_pos < -7000) scaled_pos = -7000;
    return (int16_t)scaled_pos;
}

uint8_t IR_IsLineDetected(void) {
    for (int i = 0; i < 8; i++) {
        if (last_raw_values[i] > thresholds[i]) return 1;
    }
    return 0;
}

uint8_t IR_GetActiveCount(void) {
    uint8_t count = 0;
    for (int i = 0; i < 8; i++) {
        if (last_raw_values[i] > thresholds[i]) count++;
    }
    return count;
}
