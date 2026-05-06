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
    STATE_RUNNING
} RobotState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- FINAL TUNED SETTINGS --- */
#define BASE_SPD        950
#define MAX_SPD         1000
#define FAN_SPD         600

// Authoritative High-Speed Tuning: Aggressive KP, Strong KD for overshoot prevention
#define KP              0.108f
#define KI              0.00f
#define KD              2.65f

#define RUN_DURATION_MS 2500

#define CALIB_DURATION_MS 3000
#define DOUBLE_CLICK_MS   500
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
    PID_Init(&pid, KP, KI, KD, BASE_SPD, MAX_SPD);

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
                
                // Double Click Detection
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

                if (btn_press_count >= 2) {
                    state_start_time = HAL_GetTick();
                    current_state = STATE_PRE_START_DELAY;
                }
                break;

            case STATE_PRE_START_DELAY:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF
                if (HAL_GetTick() - state_start_time >= 2000) {
                    Brushless_SetSpeed(FAN_SPD, FAN_SPD);
                    state_start_time = HAL_GetTick();
                    current_state = STATE_FAN_SPINUP;
                }
                break;

            case STATE_FAN_SPINUP:
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (HAL_GetTick() / 100) % 2);
                if (HAL_GetTick() - state_start_time >= 3000) {
                    state_start_time = HAL_GetTick();
                    current_state = STATE_RUNNING;
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
                    if (pos == -9999) {
                        // Corrected recovery: spin towards last seen side
                        if (last_side == 1) { Motor_Left(-500); Motor_Right(850); }
                        else if (last_side == -1) { Motor_Left(850); Motor_Right(-500); }
                        else { Motor_Left(350); Motor_Right(350); }
                    } else {
                        // Updated for faster side detection (2000 threshold)
                        if (pos > 2000) last_side = 1;      // LEFT
                        else if (pos < -2000) last_side = -1; // RIGHT

                        // --- DYNAMIC SPEED SCALING ---
                        int16_t current_base = BASE_SPD;
                        if (abs(pos) > 3500) {
                            current_base = BASE_SPD - 300;
                            if (current_base < 350) current_base = 350;
                        }
                        pid.base_speed = current_base;

                        int16_t l, r;
                        PID_Compute(&pid, pos, &l, &r);
                        Motor_Left(l);
                        Motor_Right(r);
                    }
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
