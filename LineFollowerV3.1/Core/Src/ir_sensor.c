#include "ir_sensor.h"
#include "tim.h"

/* Multiplexer select pins definitions */
#define MUX_S0_PORT GPIOB
#define MUX_S0_PIN  GPIO_PIN_2
#define MUX_S1_PORT GPIOB
#define MUX_S1_PIN  GPIO_PIN_8
#define MUX_S2_PORT GPIOB
#define MUX_S2_PIN  GPIO_PIN_9

/* ADC pin definition (PB1 - ADC1_IN9) */
#define ADC_PORT    GPIOB
#define ADC_PIN     GPIO_PIN_1
#define ADC_CHANNEL 9

#define IR_RAW_DETECT_THRESHOLD 250U

/* MOSFET Logic: P-Channel (Low = ON, High = OFF) */
/* When using PWM, 0% duty = ON (Low), 100% duty = OFF (High) */

/* Internal buffer for raw values */
static uint16_t last_raw_values[8];

void IR_ResetCalibration(void) {
    // Calibration removed: kept as no-op for API compatibility.
}

void IR_Calibrate(void) {
    // Calibration removed: kept as no-op for API compatibility.
}

void IR_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* MUX Control Pins */
    GPIO_InitStruct.Pin = MUX_S0_PIN | MUX_S1_PIN | MUX_S2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* ADC Pin */
    GPIO_InitStruct.Pin = ADC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Start TIM3 for IR PWM */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    IR_SetBrightness(255); // Default full brightness

    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC1->CR1 = 0; 
    ADC1->CR2 = ADC_CR2_ADON; 
    
    ADC1->SMPR2 |= (7U << 27);
}

void IR_SelectChannel(uint8_t channel) {
    HAL_GPIO_WritePin(MUX_S0_PORT, MUX_S0_PIN, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S1_PORT, MUX_S1_PIN, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_S2_PORT, MUX_S2_PIN, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // Settling delay for MUX and ADC (~20us)
    for(volatile int i=0; i<1000; i++); 
}

uint16_t IR_ReadADC(void) {
    ADC1->SQR3 = ADC_CHANNEL;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return (uint16_t)ADC1->DR;
}

void IR_SetBrightness(uint8_t brightness) {
    // P-Channel MOSFET: Low is ON, High is OFF.
    // PWM1 mode, Polarity High: 
    // Pulse = 0 -> Output Low (100% ON)
    // Pulse = ARR -> Output High (0% ON)
    uint32_t duty = (255 - brightness) * 2399 / 255;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
}

void IR_ReadAll(uint16_t *values) {
    // Read with LEDs ON (simple mode)
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
        uint16_t raw_val = last_raw_values[i];
        uint16_t weight = (raw_val > IR_RAW_DETECT_THRESHOLD) ? raw_val : 0;

        avg += (unsigned long)weight * (i * 1000U);
        sum += weight;
        if (weight > max_val) max_val = weight;
    }

    if (sum == 0 || max_val < IR_RAW_DETECT_THRESHOLD) {
        return -9999;
    }

    // Calculate raw position (0 to 7000)
    int32_t raw_pos = (int32_t)(avg / sum);
    
    // Map 0...7000 to -3500...3500
    // 0 is LEFTMOST, 7000 is RIGHTMOST.
    // -3500 = Line on LEFT, 0 = CENTER, +3500 = Line on RIGHT
    int16_t mapped_pos = (int16_t)(raw_pos - 3500);
    
    return mapped_pos;
}

uint8_t IR_IsLineDetected(void) {
    for (int i = 0; i < 8; i++) {
        if (last_raw_values[i] > 100) return 1;
    }
    return 0;
}
