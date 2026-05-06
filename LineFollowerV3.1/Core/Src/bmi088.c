#include "bmi088.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi1;

#define ACC_CS_PORT GPIOB
#define ACC_CS_PIN  GPIO_PIN_12
#define GYRO_CS_PORT GPIOB
#define GYRO_CS_PIN  GPIO_PIN_13

static void ACC_WriteReg(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);
    uint8_t buf[2] = {reg & 0x7F, data};
    HAL_SPI_Transmit(&hspi1, buf, 2, 10);
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
}

static void GYRO_WriteReg(uint8_t reg, uint8_t data) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    uint8_t buf[2] = {reg & 0x7F, data};
    HAL_SPI_Transmit(&hspi1, buf, 2, 10);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

static void ACC_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_RESET);
    uint8_t reg_addr = reg | 0x80;
    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 10);
    uint8_t dummy;
    HAL_SPI_Receive(&hspi1, &dummy, 1, 10); // Dummy byte for Accel
    HAL_SPI_Receive(&hspi1, data, len, 10);
    HAL_GPIO_WritePin(ACC_CS_PORT, ACC_CS_PIN, GPIO_PIN_SET);
}

static void GYRO_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_RESET);
    uint8_t reg_addr = reg | 0x80;
    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, 10);
    HAL_SPI_Receive(&hspi1, data, len, 10);
    HAL_GPIO_WritePin(GYRO_CS_PORT, GYRO_CS_PIN, GPIO_PIN_SET);
}

HAL_StatusTypeDef BMI088_Init(void) {
    // 1. Reset Gyro
    GYRO_WriteReg(BMI088_GYRO_SOFTRESET, 0xB6);
    HAL_Delay(50);
    
    // 2. Reset Accel
    ACC_WriteReg(BMI088_ACC_SOFTRESET, 0xB6);
    HAL_Delay(50);
    
    // 3. Turn on Accel (Active mode)
    ACC_WriteReg(BMI088_ACC_PWR_CONF, 0x00);
    HAL_Delay(10);
    ACC_WriteReg(BMI088_ACC_PWR_CTRL, 0x04);
    HAL_Delay(10);
    
    // 4. Config Ranges
    ACC_WriteReg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G);
    GYRO_WriteReg(BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_1000);
    
    return HAL_OK;
}

void BMI088_ReadAccel(float *ax, float *ay, float *az) {
    uint8_t buf[6];
    ACC_ReadRegs(BMI088_ACC_X_LSB, buf, 6);
    
    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);
    
    // Range 6G: 6 * 9.81 / 32768
    float scale = 6.0f * 9.80665f / 32768.0f;
    *ax = raw_x * scale;
    *ay = raw_y * scale;
    *az = raw_z * scale;
}

void BMI088_ReadGyro(float *gx, float *gy, float *gz) {
    uint8_t buf[6];
    GYRO_ReadRegs(BMI088_GYRO_X_LSB, buf, 6);
    
    int16_t raw_x = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t raw_y = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t raw_z = (int16_t)(buf[5] << 8 | buf[4]);
    
    // Range 1000 dps: 1000 / 32768
    float scale = 1000.0f / 32768.0f;
    *gx = raw_x * scale;
    *gy = raw_y * scale;
    *gz = raw_z * scale;
}
