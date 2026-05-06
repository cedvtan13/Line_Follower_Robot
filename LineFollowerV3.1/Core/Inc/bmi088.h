#ifndef __BMI088_H__
#define __BMI088_H__

#include "main.h"

// BMI088 Register Definitions
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_ACC_ERR_REG          0x02
#define BMI088_ACC_STATUS           0x03
#define BMI088_ACC_X_LSB            0x12
#define BMI088_ACC_CONF             0x40
#define BMI088_ACC_RANGE            0x41
#define BMI088_ACC_PWR_CONF         0x7C
#define BMI088_ACC_PWR_CTRL         0x7D
#define BMI088_ACC_SOFTRESET        0x7E

#define BMI088_GYRO_CHIP_ID         0x00
#define BMI088_GYRO_X_LSB           0x02
#define BMI088_GYRO_RANGE           0x0F
#define BMI088_GYRO_BANDWIDTH       0x10
#define BMI088_GYRO_LPM1            0x11
#define BMI088_GYRO_SOFTRESET       0x14

// Configuration Values
#define BMI088_ACC_RANGE_6G         0x01
#define BMI088_GYRO_RANGE_1000      0x01

// Driver Functions
HAL_StatusTypeDef BMI088_Init(void);
void BMI088_ReadAccel(float *ax, float *ay, float *az);
void BMI088_ReadGyro(float *gx, float *gy, float *gz);

#endif
