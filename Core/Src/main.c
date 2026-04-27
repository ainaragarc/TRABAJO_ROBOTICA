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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motores.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

#define ENC_COUNTS_PER_REV       4096.0f
#define ENCODER_DIRECTION        1.0f
#define SERVO_DIRECTION          1.0f

#define SERVO_NEUTRAL_US         1500
#define SERVO_MIN_US             1000
#define SERVO_MAX_US             2000
#define SERVO_MAX_OFFSET_US      300

#define ANGLE_START_DEG          90.0f
#define ANGLE_CENTER_DEG         90.0f
#define ANGLE_MIN_DEG            50.0f
#define ANGLE_MAX_DEG            130.0f

#define ANGLE_FRONT_DEG          130.0f
#define ANGLE_BACK_DEG           50.0f

#define TARGET_SPEED_DPS         35.0f
#define CONTROL_PERIOD_MS        10

static float KP = 8.0f;
static float KI = 1.5f;
static float KD = 0.35f;

#define INTEGRAL_LIMIT           120.0f

static float target_final_deg = ANGLE_CENTER_DEG;
static float target_ramped_deg = ANGLE_CENTER_DEG;

static float pid_integral = 0.0f;
static float prev_angle_deg = ANGLE_CENTER_DEG;

static uint32_t last_control_ms = 0;
static uint32_t state_start_ms = 0;

typedef enum
{
    ST_HOLD_CENTER_START = 0,
    ST_MOVE_FRONT,
    ST_HOLD_FRONT,
    ST_MOVE_CENTER_1,
    ST_HOLD_CENTER_1,
    ST_MOVE_BACK,
    ST_HOLD_BACK,
    ST_MOVE_CENTER_2,
    ST_HOLD_CENTER_2
} MotionState;

static MotionState state = ST_HOLD_CENTER_START;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static float clamp_float(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static int clamp_int(int x, int min, int max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

static void Servo_WriteUs(uint16_t pulse_us)
{
    pulse_us = clamp_int(pulse_us, SERVO_MIN_US, SERVO_MAX_US);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

static int32_t Encoder_GetCount(void)
{
    return (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
}

static float Encoder_GetAngleDeg(void)
{
    int32_t count = Encoder_GetCount();

    float revolutions = ((float)count) / ENC_COUNTS_PER_REV;
    float delta_deg = revolutions * 360.0f * ENCODER_DIRECTION;

    return ANGLE_START_DEG + delta_deg;
}

static void Encoder_SetCurrentPositionAs90Deg(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    prev_angle_deg = ANGLE_START_DEG;
    target_final_deg = ANGLE_CENTER_DEG;
    target_ramped_deg = ANGLE_CENTER_DEG;
    pid_integral = 0.0f;
}

static void UpdateRampedTarget(float dt_s)
{
    target_final_deg = clamp_float(target_final_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG);

    float diff = target_final_deg - target_ramped_deg;
    float max_step = TARGET_SPEED_DPS * dt_s;

    if (fabsf(diff) <= max_step)
    {
        target_ramped_deg = target_final_deg;
    }
    else
    {
        if (diff > 0.0f)
            target_ramped_deg += max_step;
        else
            target_ramped_deg -= max_step;
    }

    target_ramped_deg = clamp_float(target_ramped_deg, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
}

static void UpdateMotionState(float angle_deg)
{
    uint32_t now = HAL_GetTick();

    switch (state)
    {
        case ST_HOLD_CENTER_START:
            target_final_deg = ANGLE_CENTER_DEG;

            if (now - state_start_ms > 1500)
            {
                state = ST_MOVE_FRONT;
                state_start_ms = now;
            }
            break;

        case ST_MOVE_FRONT:
            target_final_deg = ANGLE_FRONT_DEG;

            if ((fabsf(angle_deg - ANGLE_FRONT_DEG) < 2.0f) &&
                (fabsf(target_ramped_deg - ANGLE_FRONT_DEG) < 0.5f))
            {
                state = ST_HOLD_FRONT;
                state_start_ms = now;
            }
            break;

        case ST_HOLD_FRONT:
            target_final_deg = ANGLE_FRONT_DEG;

            if (now - state_start_ms > 2000)
            {
                state = ST_MOVE_CENTER_1;
                state_start_ms = now;
            }
            break;

        case ST_MOVE_CENTER_1:
            target_final_deg = ANGLE_CENTER_DEG;

            if ((fabsf(angle_deg - ANGLE_CENTER_DEG) < 2.0f) &&
                (fabsf(target_ramped_deg - ANGLE_CENTER_DEG) < 0.5f))
            {
                state = ST_HOLD_CENTER_1;
                state_start_ms = now;
            }
            break;

        case ST_HOLD_CENTER_1:
            target_final_deg = ANGLE_CENTER_DEG;

            if (now - state_start_ms > 1000)
            {
                state = ST_MOVE_BACK;
                state_start_ms = now;
            }
            break;

        case ST_MOVE_BACK:
            target_final_deg = ANGLE_BACK_DEG;

            if ((fabsf(angle_deg - ANGLE_BACK_DEG) < 2.0f) &&
                (fabsf(target_ramped_deg - ANGLE_BACK_DEG) < 0.5f))
            {
                state = ST_HOLD_BACK;
                state_start_ms = now;
            }
            break;

        case ST_HOLD_BACK:
            target_final_deg = ANGLE_BACK_DEG;

            if (now - state_start_ms > 2000)
            {
                state = ST_MOVE_CENTER_2;
                state_start_ms = now;
            }
            break;

        case ST_MOVE_CENTER_2:
            target_final_deg = ANGLE_CENTER_DEG;

            if ((fabsf(angle_deg - ANGLE_CENTER_DEG) < 2.0f) &&
                (fabsf(target_ramped_deg - ANGLE_CENTER_DEG) < 0.5f))
            {
                state = ST_HOLD_CENTER_2;
                state_start_ms = now;
            }
            break;

        case ST_HOLD_CENTER_2:
            target_final_deg = ANGLE_CENTER_DEG;

            if (now - state_start_ms > 1000)
            {
                state = ST_MOVE_FRONT;
                state_start_ms = now;
            }
            break;

        default:
            state = ST_HOLD_CENTER_START;
            state_start_ms = now;
            break;
    }
}

static void ControlLoop_Update(void)
{
    uint32_t now = HAL_GetTick();

    if (now - last_control_ms < CONTROL_PERIOD_MS)
    {
        return;
    }

    float dt_s = (now - last_control_ms) / 1000.0f;
    last_control_ms = now;

    if ((dt_s <= 0.0f) || (dt_s > 0.1f))
    {
        dt_s = CONTROL_PERIOD_MS / 1000.0f;
    }

    float angle_deg = Encoder_GetAngleDeg();

    if ((angle_deg < 0.0f) || (angle_deg > 180.0f))
    {
        Servo_WriteUs(SERVO_NEUTRAL_US);
        pid_integral = 0.0f;
        return;
    }

    UpdateMotionState(angle_deg);
    UpdateRampedTarget(dt_s);

    float velocity_dps = (angle_deg - prev_angle_deg) / dt_s;
    prev_angle_deg = angle_deg;

    float error_deg = target_ramped_deg - angle_deg;

    pid_integral += error_deg * dt_s;
    pid_integral = clamp_float(pid_integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float u = KP * error_deg + KI * pid_integral - KD * velocity_dps;

    u = clamp_float(u, -SERVO_MAX_OFFSET_US, SERVO_MAX_OFFSET_US);

    if ((angle_deg <= ANGLE_MIN_DEG) && (u < 0.0f))
    {
        u = 0.0f;
        pid_integral = 0.0f;
    }

    if ((angle_deg >= ANGLE_MAX_DEG) && (u > 0.0f))
    {
        u = 0.0f;
        pid_integral = 0.0f;
    }

    int pulse_us = SERVO_NEUTRAL_US + (int)(SERVO_DIRECTION * u);

    pulse_us = clamp_int(pulse_us, SERVO_MIN_US, SERVO_MAX_US);

    Servo_WriteUs((uint16_t)pulse_us);
}

//-------------------------------------------------------------------------------------------------

static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000U);
  while ((DWT->CYCCNT - start) < ticks);
}

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);	// Posicion del motor 1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	// Posicion del motor 2
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);	// Posicion del motor del revolver

  HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_SET);     // habilitado siempre durante pruebas
  HAL_GPIO_WritePin(GPIOB, STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_RESET);

  DWT_Init();

  uint32_t stepLowUs = 2500;    // arranque suave
  uint32_t accelCount = 0;
  int8_t lastDir = 0;           // -1, +1, 0

//------------------------------------------------------------

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  /*
     Antes de arrancar:
     coloca físicamente el eslabón en 90 grados.
  */
  Encoder_SetCurrentPositionAs90Deg();

  Servo_WriteUs(SERVO_NEUTRAL_US);

  HAL_Delay(1000);

  state = ST_HOLD_CENTER_START;
  state_start_ms = HAL_GetTick();
  last_control_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  ControlLoop_Update();

/*
//---------------PRUEBA MOTOR PASO A PASO--------------------

	  uint8_t btn1 = (HAL_GPIO_ReadPin(GPIOB, Paso_dcha_Pin) == GPIO_PIN_RESET);
	  uint8_t btn2 = (HAL_GPIO_ReadPin(GPIOB, Paso_Izq_Pin) == GPIO_PIN_RESET);

	  if (btn1 && !btn2)
	  {
	    if (lastDir != -1)
	    {
	      HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_RESET);
	      delay_us(2000);        // pequeña pausa al cambiar de sentido
	      stepLowUs = 2500;
	      accelCount = 0;
	      lastDir = -1;
	    }

	    HAL_GPIO_WritePin(GPIOB, STEP_Pin, GPIO_PIN_SET);
	    delay_us(20);            // pulso alto amplio
	    HAL_GPIO_WritePin(GPIOB, STEP_Pin, GPIO_PIN_RESET);
	    delay_us(stepLowUs);

	    accelCount++;
	    if (accelCount >= 25 && stepLowUs > 800)
	    {
	      stepLowUs -= 50;       // acelera poco a poco
	      accelCount = 0;
	    }
	  }
	  else if (btn2 && !btn1)
	  {
	    if (lastDir != 1)
	    {
	      HAL_GPIO_WritePin(GPIOB, DIR_Pin, GPIO_PIN_SET);
	      delay_us(2000);
	      stepLowUs = 2500;
	      accelCount = 0;
	      lastDir = 1;
	    }

	    HAL_GPIO_WritePin(GPIOB, STEP_Pin, GPIO_PIN_SET);
	    delay_us(20);
	    HAL_GPIO_WritePin(GPIOB, STEP_Pin, GPIO_PIN_RESET);
	    delay_us(stepLowUs);

	    accelCount++;
	    if (accelCount >= 25 && stepLowUs > 800)
	    {
	      stepLowUs -= 50;
	      accelCount = 0;
	    }
	  }
	  else
	  {
	    HAL_GPIO_WritePin(GPIOB, STEP_Pin, GPIO_PIN_RESET);
	    stepLowUs = 2500;
	    accelCount = 0;
	    lastDir = 0;
	  }
//----------------------------------------------------------------
*/



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP_Pin|DIR_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_Pin DIR_Pin EN_Pin */
  GPIO_InitStruct.Pin = STEP_Pin|DIR_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Paso_Izq_Pin Paso_dcha_Pin */
  GPIO_InitStruct.Pin = Paso_Izq_Pin|Paso_dcha_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
