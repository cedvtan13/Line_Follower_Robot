/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- TUNING PARAMETERS ---
#define BASE_SPEED        650
#define MAX_SPEED         1000
#define DOWNFORCE_SPEED   550   


#define PID_KP            2.10f
#define PID_KI            0.0f
#define PID_KD            2.60f

#define CORNER_MID_THRESHOLD     1300
#define CORNER_HARD_THRESHOLD    2000
#define CORNER_PIVOT_THRESHOLD   3050
#define CORNER_MID_RATE_THRESHOLD   500
#define CORNER_HARD_RATE_THRESHOLD  800
#define CORNER_MID_BASE_SPEED    360
#define CORNER_HARD_BASE_SPEED   220
#define CORNER_PIVOT_OUTER_SPEED 560
#define CORNER_PIVOT_INNER_SPEED 0

#define RECOVERY_SPEED    400
#define RECOVERY_TURN_HINT_THRESHOLD  700
#define RECOVERY_ARC_SPEED            320
#define RECOVERY_PIVOT_OUTER_SPEED    420
#define RECOVERY_PIVOT_INNER_SPEED      0
#define RECOVERY_PIVOT_DELAY_MS       120
#define RECOVERY_TIMEOUT  2000

#define LINE_LOCK_REQUIRED_SAMPLES       4
#define STARTUP_ACQUIRE_MS             450
#define STARTUP_ACQUIRE_SPEED          240

#define RUN_DURATION_MS   4600

#define START_DELAY_MS    2000  // Wait before props
#define SPINUP_DELAY_MS   3000  // Wait for props to stabilize
// -------------------------
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ir_values[8];
int16_t line_position = 0;
int16_t last_line_position = 0;
int8_t recovery_turn_dir = 0;  // +1 = search right, -1 = search left, 0 = unknown
uint8_t line_lock_count = 0;
uint8_t line_locked = 0;

PID_Config line_pid;

// State Machine
typedef enum {
    STATE_STANDBY,
    STATE_DELAY,    // Wait before props
    STATE_SPINUP,   // Props on, wheels off
    STATE_RUN       // Everything on
} RobotState;

RobotState current_state = STATE_STANDBY;

// Timing variables
uint32_t run_start_time = 0;
uint32_t delay_start_time = 0;
uint32_t spinup_start_time = 0;
uint32_t lost_line_start_time = 0;

// Button state variables
uint32_t last_button_time = 0;
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
  /* 1. Initialize Drivers */
  IR_Init();
  Motor_Init();
  Brushless_Init();
  
  /* 2. PID Settings */
  PID_Init(&line_pid, PID_KP, PID_KI, PID_KD, BASE_SPEED, MAX_SPEED);

  /* 3. Initial Stop */
  Motor_Stop();
  Brushless_SetSpeed(0, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      // 1. Button Logic (PC14) - single click to start, click again to stop
      uint8_t btn_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14);
      static uint8_t last_btn_state = 0;

      if (btn_state == GPIO_PIN_SET && last_btn_state == 0 && (HAL_GetTick() - last_button_time > 50)) {
          last_button_time = HAL_GetTick();
          if (current_state != STATE_STANDBY) {
              // Any button press during operation stops the system
              current_state = STATE_STANDBY;
              line_lock_count = 0;
              line_locked = 0;
              recovery_turn_dir = 0;
              Motor_Stop();
              Brushless_SetSpeed(0, 0);
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          } else {
              line_lock_count = 0;
              line_locked = 0;
              recovery_turn_dir = 0;
              current_state = STATE_DELAY;
              delay_start_time = HAL_GetTick();
          }
      }
      last_btn_state = btn_state;

      // 2. Control Loop
      if (current_state == STATE_DELAY) {
          Motor_Stop();
          Brushless_SetSpeed(0, 0); 
          if (HAL_GetTick() - delay_start_time >= START_DELAY_MS) {
              current_state = STATE_SPINUP;
              spinup_start_time = HAL_GetTick();
              Brushless_SetSpeed(DOWNFORCE_SPEED, DOWNFORCE_SPEED);
          }
          if ((HAL_GetTick() / 200) % 2) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          
      } else if (current_state == STATE_SPINUP) {
          Motor_Stop();
          Brushless_SetSpeed(DOWNFORCE_SPEED, DOWNFORCE_SPEED);
          if (HAL_GetTick() - spinup_start_time >= SPINUP_DELAY_MS) {
              current_state = STATE_RUN;
              run_start_time = HAL_GetTick();
              lost_line_start_time = 0;
              line_lock_count = 0;
              line_locked = 0;
              recovery_turn_dir = 0;
              last_line_position = 0;
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          }
          if ((HAL_GetTick() / 50) % 2) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
          
      } else if (current_state == STATE_RUN) {
          // Timer Check: Automatic stop after 5 seconds
          if (HAL_GetTick() - run_start_time >= RUN_DURATION_MS) {
              current_state = STATE_STANDBY;
              line_lock_count = 0;
              line_locked = 0;
              recovery_turn_dir = 0;
              Motor_Stop();
              Brushless_SetSpeed(0, 0);
              continue;
          }

          IR_ReadAll(ir_values);
          int16_t position = IR_GetLinePosition();
          
          if (position == -9999) {
              if (lost_line_start_time == 0) {
                  lost_line_start_time = HAL_GetTick();
              }

              uint32_t lost_ms = HAL_GetTick() - lost_line_start_time;
              if (!line_locked) {
                  Motor_Left(STARTUP_ACQUIRE_SPEED);
                  Motor_Right(STARTUP_ACQUIRE_SPEED);
                  Brushless_SetSpeed(DOWNFORCE_SPEED, DOWNFORCE_SPEED);

                  if (lost_ms >= STARTUP_ACQUIRE_MS) {
                      current_state = STATE_STANDBY;
                      line_lock_count = 0;
                      line_locked = 0;
                      recovery_turn_dir = 0;
                      Motor_Stop();
                      Brushless_SetSpeed(0, 0);
                  }
                  continue;
              }
              if (recovery_turn_dir == 0) {
                  recovery_turn_dir = (last_line_position >= 0) ? 1 : -1;
              }
              if (recovery_turn_dir > 0) {
                  if (lost_ms < RECOVERY_PIVOT_DELAY_MS) {
                      Motor_Left(RECOVERY_ARC_SPEED);
                      Motor_Right(0);
                  } else {
                      Motor_Left(RECOVERY_PIVOT_OUTER_SPEED);
                      Motor_Right(RECOVERY_PIVOT_INNER_SPEED);
                  }
              } else {
                  if (lost_ms < RECOVERY_PIVOT_DELAY_MS) {
                      Motor_Left(0);
                      Motor_Right(RECOVERY_ARC_SPEED);
                  } else {
                      Motor_Left(RECOVERY_PIVOT_INNER_SPEED);
                      Motor_Right(RECOVERY_PIVOT_OUTER_SPEED);
                  }
              }

              if (lost_ms >= RECOVERY_TIMEOUT) {
                  current_state = STATE_STANDBY;
                  Motor_Stop();
                  Brushless_SetSpeed(0, 0);
              }
              continue;
          }
          
          line_position = position;
          int16_t pos_delta = line_position - last_line_position;
          int16_t abs_delta = (pos_delta >= 0) ? pos_delta : -pos_delta;
          lost_line_start_time = 0;
          if (line_lock_count < 255) {
              line_lock_count++;
          }
          if (line_lock_count >= LINE_LOCK_REQUIRED_SAMPLES) {
              line_locked = 1;
          }

          // Track the latest turn direction so loss recovery turns the correct way.
          if (line_position >= RECOVERY_TURN_HINT_THRESHOLD) {
              recovery_turn_dir = 1;
          } else if (line_position <= -RECOVERY_TURN_HINT_THRESHOLD) {
              recovery_turn_dir = -1;
          } else if (abs_delta >= CORNER_MID_RATE_THRESHOLD) {
              recovery_turn_dir = (pos_delta >= 0) ? 1 : -1;
          }

          // Slow down in sharper corners to reduce overshoot.
          int16_t abs_pos = (line_position >= 0) ? line_position : -line_position;

          if (abs_pos >= CORNER_PIVOT_THRESHOLD) {
              if (line_position > 0) {
                  Motor_Left(CORNER_PIVOT_OUTER_SPEED);
                  Motor_Right(CORNER_PIVOT_INNER_SPEED);
              } else {
                  Motor_Left(CORNER_PIVOT_INNER_SPEED);
                  Motor_Right(CORNER_PIVOT_OUTER_SPEED);
              }
              Brushless_SetSpeed(DOWNFORCE_SPEED, DOWNFORCE_SPEED);
              last_line_position = line_position;
              continue;
          }

          if (abs_pos >= CORNER_HARD_THRESHOLD || abs_delta >= CORNER_HARD_RATE_THRESHOLD) {
              line_pid.base_speed = CORNER_HARD_BASE_SPEED;
          } else if (abs_pos >= CORNER_MID_THRESHOLD || abs_delta >= CORNER_MID_RATE_THRESHOLD) {
              line_pid.base_speed = CORNER_MID_BASE_SPEED;
          } else {
              line_pid.base_speed = BASE_SPEED;
          }

          int16_t target_left, target_right;
          PID_Compute(&line_pid, line_position, &target_left, &target_right);
          last_line_position = line_position;
          
          Motor_Left(target_left);
          Motor_Right(target_right);
          Brushless_SetSpeed(DOWNFORCE_SPEED, DOWNFORCE_SPEED);
          
      } else {
          // STANDBY
          line_lock_count = 0;
          line_locked = 0;
          recovery_turn_dir = 0;
          Motor_Stop();
          Brushless_SetSpeed(0, 0);
          if ((HAL_GetTick() / 1000) % 2) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          else HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
      }

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
     ex: printf("Wrong parameters value: file %s on line %d\\r\\n\", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
