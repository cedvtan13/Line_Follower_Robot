/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ir_sensor.h"
#include "motor.h"
#include "pid.h"
#include "brushless.h"
#include "bmi088.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_STANDBY,
    STATE_CALIBRATING,
    STATE_READY,
    STATE_PRE_START_DELAY,
    STATE_FAN_SPINUP,
    STATE_RUNNING,
    STATE_MAPPING,
    STATE_FAST_RUN
} RobotState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- MICROMOUSE-INSPIRED SPEED SETTINGS --- */
#define NOMINAL_SPD     850
#define BOOST_SPD       1000
#define MAX_SPD         1000
#define FAN_SPD         650 
#define MAP_BASE_SPD    250
#define MAP_FAN_SPD     0
#define FAST_FAN_SPD    600

// Retuned for Extreme Speed: Higher KD to handle the momentum
#define KP              0.115f
#define KI              0.00f
#define KD              2.95f

#define RUN_DURATION_MS 4000

#define CALIB_DURATION_MS 3000
#define DOUBLE_CLICK_MS   500

#define MAPPING_DURATION_MS  10000
#define FAST_RUN_DURATION_MS 3500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
PID_Config pid;
uint16_t sensors[8];
uint32_t run_start_time = 0;
int8_t last_side = 0;
RobotState current_state = STATE_STANDBY;

// Button logic
uint32_t last_btn_press_time = 0;
uint8_t btn_press_count = 0;

// Micromouse-inspired logic variables
uint32_t centered_start_time = 0;
int16_t last_line_pos = 0;

// Track Mapping Buffer (30 seconds at 10ms = 3000 bytes)
uint8_t map_buffer[3000];
uint32_t map_index = 0;
uint32_t map_max_index = 0;
uint32_t last_map_tick = 0;
uint32_t lost_start_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    IR_Init();
    Motor_Init();
    Brushless_Init();
    BMI088_Init();
    PID_Init(&pid, KP, KI, KD, NOMINAL_SPD, MAX_SPD);

    Motor_Stop();
    Brushless_SetSpeed(0, 0);

    uint32_t state_start_time = 0;
    uint32_t last_print = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        IR_ReadAll(sensors);
        int16_t pos = IR_GetLinePosition();
        uint8_t count = IR_GetActiveCount();

        // Categorize and Print raw values to SWO every 500ms
        if (HAL_GetTick() - last_print > 500) {
            const char* zone = "LOST";
            if (pos != -9999) {
                int16_t abs_pos = (pos >= 0) ? pos : -pos;
                if (abs_pos < 500) zone = "CENTER";
                else if (abs_pos < 2000) zone = (pos > 0) ? "MID-LEFT" : "MID-RIGHT";
                else if (abs_pos < 4500) zone = (pos > 0) ? "NORM-LEFT" : "NORM-RIGHT";
                else zone = (pos > 0) ? "HARD-LEFT" : "HARD-RIGHT";
            }
            printf("[%s] Pos: %5d | State: %d | Run: 750\r\n", zone, pos, current_state);
            last_print = HAL_GetTick();
        }

        switch (current_state) {
            case STATE_STANDBY:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (HAL_GetTick() / 1000) % 2); // Slow blink
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
                    HAL_Delay(200); // Debounce
                    IR_StartCalibration();
                    state_start_time = HAL_GetTick();
                    current_state = STATE_CALIBRATING;
                }
                break;

            case STATE_CALIBRATING:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (HAL_GetTick() / 200) % 2); // Fast blink (200ms)
                IR_UpdateCalibration();
                if (HAL_GetTick() - state_start_time >= CALIB_DURATION_MS) {
                    IR_FinishCalibration();
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
                    current_state = STATE_READY;
                    btn_press_count = 0;
                }
                break;

            case STATE_READY:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
                
                // Button Click Detection
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET) {
                    HAL_Delay(100); // Debounce
                    uint32_t now = HAL_GetTick();
                    if (now - last_btn_press_time < DOUBLE_CLICK_MS) {
                        btn_press_count++;
                    } else {
                        btn_press_count = 1;
                    }
                    last_btn_press_time = now;
                    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_SET); // Wait for release
                }

                // Start sequence after a short delay to allow for multiple clicks
                if (btn_press_count > 0 && (HAL_GetTick() - last_btn_press_time > DOUBLE_CLICK_MS)) {
                    state_start_time = HAL_GetTick();
                    current_state = STATE_PRE_START_DELAY;
                }
                break;

            case STATE_PRE_START_DELAY:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF
                if (HAL_GetTick() - state_start_time >= 2000) {
                    uint16_t current_fan = FAN_SPD;
                    if (btn_press_count == 2) current_fan = MAP_FAN_SPD;
                    else if (btn_press_count >= 3) current_fan = FAST_FAN_SPD;
                    
                    Brushless_SetSpeed(current_fan, current_fan);
                    state_start_time = HAL_GetTick();
                    current_state = STATE_FAN_SPINUP;
                }
                break;

            case STATE_FAN_SPINUP:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (HAL_GetTick() / 100) % 2);
                if (HAL_GetTick() - state_start_time >= 3000) {
                    state_start_time = HAL_GetTick();
                    if (btn_press_count == 1) current_state = STATE_RUNNING;
                    else if (btn_press_count == 2) {
                        current_state = STATE_MAPPING;
                        map_index = 0;
                        last_map_tick = HAL_GetTick();
                    }
                    else if (btn_press_count >= 3) current_state = STATE_FAST_RUN;
                }
                break;

            case STATE_RUNNING:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

                if (HAL_GetTick() - state_start_time >= RUN_DURATION_MS) {
                    Motor_Stop();
                    Brushless_SetSpeed(0, 0);
                    current_state = STATE_STANDBY;
                    HAL_Delay(1000);
                } else {
                    // --- IMU ASSISTANCE ---
                    float gx, gy, gz, ax, ay, az;
                    BMI088_ReadGyro(&gx, &gy, &gz);
                    BMI088_ReadAccel(&ax, &ay, &az);
                    
                    if (pos == -9999) {
                        // Corrected recovery: spin towards last seen side
                        if (last_side == 1) { Motor_Left(-500); Motor_Right(850); }
                        else if (last_side == -1) { Motor_Left(850); Motor_Right(-500); }
                        else { Motor_Left(350); Motor_Right(350); }
                    } else {
                        // Updated for faster side detection (2000 threshold)
                        if (pos > 2000) last_side = 1;      // LEFT
                        else if (pos < -2000) last_side = -1; // RIGHT

                        // --- MICROMOUSE VELOCITY PROFILING ---
                        int16_t current_base = NOMINAL_SPD;

                        // 1. Straight-Line Boost
                        if (abs(pos) < 500) {
                            if (centered_start_time == 0) centered_start_time = HAL_GetTick();
                            uint32_t centered_dur = HAL_GetTick() - centered_start_time;
                            if (centered_dur > 150) {
                                // Ramp from NOMINAL to BOOST over 300ms
                                int16_t ramp = (int16_t)((centered_dur - 150) * (BOOST_SPD - NOMINAL_SPD) / 300);
                                current_base = NOMINAL_SPD + ramp;
                                if (current_base > BOOST_SPD) current_base = BOOST_SPD;
                            }
                        } else {
                            centered_start_time = 0;
                            current_base = NOMINAL_SPD;
                        }
                        
                        // 2. Predictive Rate Braking (Lookahead)
                        int16_t pos_delta = abs(pos - last_line_pos);
                        if (pos_delta > 800) { // Line is moving fast!
                            int16_t predictive_brake = (pos_delta - 800) * 1.2f;
                            current_base -= predictive_brake;
                        }
                        last_line_pos = pos;

                        // 3. IR-based scaling (fallback/safety)
                        if (abs(pos) > 3500) {
                            current_base -= 200;
                        }
                        
                        // 4. IMU-based Yaw Rate Braking
                        float abs_yaw = (gz > 0) ? gz : -gz;
                        if (abs_yaw > 300.0f) {
                            int16_t yaw_reduction = (int16_t)((abs_yaw - 300.0f) * 0.8f);
                            current_base -= yaw_reduction;
                        }

                        if (current_base < 350) current_base = 350;
                        pid.base_speed = current_base;

                        int16_t l, r;
                        PID_Compute(&pid, pos, &l, &r);
                        Motor_Left(l);
                        Motor_Right(r);
                    }
                }
                break;

            case STATE_MAPPING:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (HAL_GetTick() / 50) % 2); // Super fast blink

                if (HAL_GetTick() - state_start_time >= MAPPING_DURATION_MS || pos == -9999) {
                    if (pos == -9999) {
                        if (lost_start_tick == 0) lost_start_tick = HAL_GetTick();
                        if (HAL_GetTick() - lost_start_tick < 200) goto mapping_active;
                    }
                    // End mapping
                    map_max_index = map_index;
                    Motor_Stop();
                    Brushless_SetSpeed(0, 0);
                    current_state = STATE_STANDBY;
                    printf("MAPPING DONE. Total Frames: %lu\r\n", map_max_index);
                    HAL_Delay(1000);
                    break;
                } else {
                    lost_start_tick = 0;
                }

            mapping_active:
                // PID for slow, stable mapping
                pid.base_speed = MAP_BASE_SPD;
                int16_t l_map, r_map;
                PID_Compute(&pid, pos, &l_map, &r_map);
                Motor_Left(l_map);
                Motor_Right(r_map);

                // Record data every 10ms
                if (HAL_GetTick() - last_map_tick >= 10) {
                    if (map_index < 3000) {
                        map_buffer[map_index++] = (uint8_t)(abs(pos) / 28);
                    }
                    last_map_tick = HAL_GetTick();
                }
                break;

            case STATE_FAST_RUN:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON
                
                if (HAL_GetTick() - state_start_time >= FAST_RUN_DURATION_MS) {
                    Motor_Stop();
                    Brushless_SetSpeed(0, 0);
                    current_state = STATE_STANDBY;
                    HAL_Delay(1000);
                    break;
                }

                if (pos == -9999) {
                    if (last_side == 1) { Motor_Left(-500); Motor_Right(850); }
                    else if (last_side == -1) { Motor_Left(850); Motor_Right(-500); }
                    else { Motor_Left(350); Motor_Right(350); }
                } else {
                    if (pos > 2000) last_side = 1;
                    else if (pos < -2000) last_side = -1;

                    // --- MAP-ASSISTED SPEED CONTROL ---
                    int16_t current_base = NOMINAL_SPD;

                    // Estimate current map index based on time and speed ratio
                    // (Note: This is an approximation since we don't have encoders)
                    uint32_t elapsed = HAL_GetTick() - state_start_time;
                    float speed_ratio = (float)NOMINAL_SPD / (float)MAP_BASE_SPD;
                    uint32_t estimated_index = (uint32_t)((float)elapsed / 10.0f * speed_ratio);
                    
                    // Lookahead: 150ms ahead (15 frames)
                    uint32_t lookahead_idx = estimated_index + 15;
                    
                    if (lookahead_idx < map_max_index) {
                        uint8_t future_curve = map_buffer[lookahead_idx];
                        if (future_curve > 80) { // Equivalent to abs(pos) > 2240
                            current_base = 500; // Predictive Braking
                        } else if (future_curve < 20) { // Straightaway
                            current_base = BOOST_SPD;
                        }
                    }

                    // IMU Safety (Always Active)
                    float gx, gy, gz, ax, ay, az;
                    BMI088_ReadGyro(&gx, &gy, &gz);
                    float abs_yaw = (gz > 0) ? gz : -gz;
                    if (abs_yaw > 300.0f) {
                        current_base -= (int16_t)((abs_yaw - 300.0f) * 0.8f);
                    }

                    if (current_base < 350) current_base = 350;
                    pid.base_speed = current_base;

                    int16_t l_fast, r_fast;
                    PID_Compute(&pid, pos, &l_fast, &r_fast);
                    Motor_Left(l_fast);
                    Motor_Right(r_fast);
                }
                break;
        }
        HAL_Delay(5);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* --- SWO PRINTF REDIRECTION --- */
int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
