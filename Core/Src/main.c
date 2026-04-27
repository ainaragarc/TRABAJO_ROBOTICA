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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

//------------------stepper-------------------------------------------

#define STEP_TIM                htim4
#define STEP_TIM_CHANNEL        TIM_CHANNEL_1

#define STEP_DIR_PORT           GPIOB
#define STEP_DIR_PIN            GPIO_PIN_7

#define STEP_EN_PORT            GPIOB
#define STEP_EN_PIN             GPIO_PIN_8

#define DIR_POSITIVE            GPIO_PIN_SET
#define DIR_NEGATIVE            GPIO_PIN_RESET

// TIM4 está en APB1. Con tu clock: APB1 timer clock = 50 MHz.
// Prescaler = 49 -> tick de 1 MHz -> 1 us.
#define STEP_TIMER_TICK_HZ      1000000UL

// Mecánica
#define STEPS_PER_REV           200.0f
#define MM_PER_REV              8.0f
#define STEPS_PER_MM            (STEPS_PER_REV / MM_PER_REV)   // 25 pasos/mm

// Corredera
#define SLIDER_LENGTH_MM        440.0f

// Velocidad mínima deseada: 1 cm/s = 10 mm/s
#define MIN_SPEED_MM_S          10.0f

//-----------------------------------------------------------------------


//------------------FCs-------------------------------------------

#define LIMIT_MIN_PORT          GPIOA
#define LIMIT_MIN_PIN           GPIO_PIN_8

#define LIMIT_MAX_PORT          GPIOA
#define LIMIT_MAX_PIN           GPIO_PIN_9

volatile uint8_t limit_min_hit = 0;
volatile uint8_t limit_max_hit = 0;

volatile uint8_t stepper_is_running = 0;
volatile uint8_t stepper_current_dir = 0;

// Para antirrebote básico
volatile uint32_t last_limit_min_ms = 0;
volatile uint32_t last_limit_max_ms = 0;

//-----------------------------------------------------------------------


//------------------Encoder_Base-------------------------------------------

#define ENC_TIM                 htim3

volatile int32_t encoder_position_counts = 0;
static uint16_t encoder_last_raw = 0;

volatile int32_t encoder_total_counts = 0;
volatile int32_t encoder_mid_counts = 0;

volatile uint8_t homing_done = 0;

// Debug
volatile uint16_t dbg_enc_raw = 0;
volatile int32_t dbg_enc_pos = 0;
volatile int32_t dbg_enc_total = 0;
volatile int32_t dbg_enc_mid = 0;
volatile uint8_t dbg_homing_state = 0;

//-----------------------------------------------------------------------

//------------------Maquina estados Homing-------------------------------------------
typedef enum
{
    HOMING_START = 0,
    HOMING_MOVE_TO_MIN,
    HOMING_AT_MIN,
    HOMING_MOVE_TO_MAX,
    HOMING_AT_MAX,
    HOMING_MOVE_TO_MID,
    HOMING_DONE
} HomingState;

volatile HomingState homing_state = HOMING_START;

#define BASE_SPEED_MM_S         40.0f

// Según tu prueba anterior, esto debería estar así.
// Si ves que va al lado contrario, los intercambiamos.
#define DIR_TO_MIN              0
#define DIR_TO_MAX              1

// Margen de parada en cuentas de encoder para el punto medio.
#define MID_TOLERANCE_COUNTS    5

//-----------------------------------------------------------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-----------------------------stepper------------------------------------------

static void Stepper_Enable(void)
{
    // A4988: ENABLE activo en LOW
    HAL_GPIO_WritePin(STEP_EN_PORT, STEP_EN_PIN, GPIO_PIN_RESET);
}

static void Stepper_Disable(void)
{
    HAL_GPIO_WritePin(STEP_EN_PORT, STEP_EN_PIN, GPIO_PIN_SET);
}

static void Stepper_SetDir(uint8_t dir)
{
    if (dir)
    {
        HAL_GPIO_WritePin(STEP_DIR_PORT, STEP_DIR_PIN, DIR_POSITIVE);
    }
    else
    {
        HAL_GPIO_WritePin(STEP_DIR_PORT, STEP_DIR_PIN, DIR_NEGATIVE);
    }
}

static uint32_t Stepper_MmS_To_StepsS(float speed_mm_s)
{
    if (speed_mm_s < MIN_SPEED_MM_S)
    {
        speed_mm_s = MIN_SPEED_MM_S;
    }

    return (uint32_t)(speed_mm_s * STEPS_PER_MM);
}

static void Stepper_SetSpeedStepsPerSec(uint32_t steps_per_sec)
{
    if (steps_per_sec == 0)
    {
        HAL_TIM_PWM_Stop(&STEP_TIM, STEP_TIM_CHANNEL);
        return;
    }

    uint32_t period_us = STEP_TIMER_TICK_HZ / steps_per_sec;

    if (period_us < 20)
    {
        period_us = 20;
    }

    uint32_t arr = period_us - 1;
    uint32_t ccr = period_us / 2;

    __HAL_TIM_SET_AUTORELOAD(&STEP_TIM, arr);
    __HAL_TIM_SET_COMPARE(&STEP_TIM, STEP_TIM_CHANNEL, ccr);
    __HAL_TIM_SET_COUNTER(&STEP_TIM, 0);
}

static void Stepper_StartStepsPerSec(uint32_t steps_per_sec, uint8_t dir)
{
    Stepper_SetDir(dir);
    Stepper_SetSpeedStepsPerSec(steps_per_sec);

    stepper_current_dir = dir;
    stepper_is_running = 1;

    Stepper_Enable();

    HAL_TIM_PWM_Start(&STEP_TIM, STEP_TIM_CHANNEL);
}

static void Stepper_StartMmS(float speed_mm_s, uint8_t dir)
{
    uint32_t steps_per_sec = Stepper_MmS_To_StepsS(speed_mm_s);
    Stepper_StartStepsPerSec(steps_per_sec, dir);
}

static void Stepper_StopHold(void)
{
    HAL_TIM_PWM_Stop(&STEP_TIM, STEP_TIM_CHANNEL);

    stepper_is_running = 0;

    // Mantiene par
    Stepper_Enable();
}

//-----------------------------------------------------------------------

//-----------------------------FCs------------------------------------------
static uint8_t LimitMin_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_MIN_PORT, LIMIT_MIN_PIN) == GPIO_PIN_RESET);
}

static uint8_t LimitMax_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_MAX_PORT, LIMIT_MAX_PIN) == GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == LIMIT_MIN_PIN)
    {
        if (now - last_limit_min_ms < 50)
        {
            return;
        }

        last_limit_min_ms = now;

        if (LimitMin_IsPressed())
        {
            limit_min_hit = 1;

            // Suponemos dir = 0 hacia MIN
            if (stepper_is_running && stepper_current_dir == 0)
            {
                Stepper_StopHold();
            }
        }
    }

    if (GPIO_Pin == LIMIT_MAX_PIN)
    {
        if (now - last_limit_max_ms < 50)
        {
            return;
        }

        last_limit_max_ms = now;

        if (LimitMax_IsPressed())
        {
            limit_max_hit = 1;

            // Suponemos dir = 1 hacia MAX
            if (stepper_is_running && stepper_current_dir == 1)
            {
                Stepper_StopHold();
            }
        }
    }
}


//-----------------------------------------------------------------------

//------------------Encoder_Base-------------------------------------------
static void Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&ENC_TIM, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&ENC_TIM, 0);

    encoder_last_raw = 0;
    encoder_position_counts = 0;
}

static void Encoder_ResetPosition(void)
{
    __HAL_TIM_SET_COUNTER(&ENC_TIM, 0);

    encoder_last_raw = 0;
    encoder_position_counts = 0;
}

static void Encoder_Update(void)
{
    uint16_t raw_now = (uint16_t)__HAL_TIM_GET_COUNTER(&ENC_TIM);

    /*
       Resta con int16_t para gestionar automáticamente overflow de 16 bits.
       Ejemplo:
       65535 -> 0 cuenta como +1
       0 -> 65535 cuenta como -1
    */
    int16_t delta = (int16_t)(raw_now - encoder_last_raw);

    encoder_last_raw = raw_now;

    encoder_position_counts += delta;

    dbg_enc_raw = raw_now;
    dbg_enc_pos = encoder_position_counts;
    dbg_enc_total = encoder_total_counts;
    dbg_enc_mid = encoder_mid_counts;
}

//-----------------------------------------------------------------------

//------------------Maquina estados Homing-------------------------------------------

static void Homing_Update(void)
{
    Encoder_Update();

    dbg_homing_state = (uint8_t)homing_state;

    switch (homing_state)
    {
        case HOMING_START:
        {
            homing_done = 0;

            limit_min_hit = 0;
            limit_max_hit = 0;

            Stepper_StartMmS(BASE_SPEED_MM_S, DIR_TO_MIN);

            homing_state = HOMING_MOVE_TO_MIN;
            break;
        }

        case HOMING_MOVE_TO_MIN:
        {
            if (LimitMin_IsPressed() || limit_min_hit)
            {
                Stepper_StopHold();
                HAL_Delay(200);

                Encoder_ResetPosition();

                limit_min_hit = 0;
                limit_max_hit = 0;

                homing_state = HOMING_AT_MIN;
            }
            break;
        }

        case HOMING_AT_MIN:
        {
            HAL_Delay(300);

            Stepper_StartMmS(BASE_SPEED_MM_S, DIR_TO_MAX);

            homing_state = HOMING_MOVE_TO_MAX;
            break;
        }

        case HOMING_MOVE_TO_MAX:
        {
            if (LimitMax_IsPressed() || limit_max_hit)
            {
                Stepper_StopHold();
                HAL_Delay(200);

                Encoder_Update();

                encoder_total_counts = encoder_position_counts;

                /*
                   Si el encoder cuenta negativo al ir hacia MAX,
                   lo convertimos a positivo.
                */
                if (encoder_total_counts < 0)
                {
                    encoder_total_counts = -encoder_total_counts;
                    encoder_position_counts = encoder_total_counts;
                }

                encoder_mid_counts = encoder_total_counts / 2;

                limit_min_hit = 0;
                limit_max_hit = 0;

                homing_state = HOMING_AT_MAX;
            }
            break;
        }

        case HOMING_AT_MAX:
        {
            HAL_Delay(300);

            /*
               Estamos en MAX y queremos ir al punto medio.
               Por tanto, vamos hacia MIN hasta que la posición sea <= mid.
            */
            Stepper_StartMmS(BASE_SPEED_MM_S, DIR_TO_MIN);

            homing_state = HOMING_MOVE_TO_MID;
            break;
        }

        case HOMING_MOVE_TO_MID:
        {
            /*
               Caso normal:
               MIN = 0
               MAX = encoder_total_counts
               vamos desde MAX hacia MID, así que encoder_position_counts baja.
            */
            if (encoder_position_counts <= encoder_mid_counts + MID_TOLERANCE_COUNTS)
            {
                Stepper_StopHold();

                homing_done = 1;
                homing_state = HOMING_DONE;
            }

            /*
               Seguridad extra: si toca MIN antes de llegar al medio, parar.
            */
            if (LimitMin_IsPressed() || limit_min_hit)
            {
                Stepper_StopHold();

                homing_done = 0;
                homing_state = HOMING_DONE;
            }

            break;
        }

        case HOMING_DONE:
        {
            Stepper_StopHold();
            break;
        }

        default:
        {
            Stepper_StopHold();
            homing_state = HOMING_DONE;
            break;
        }
    }
}

//-----------------------------------------------------------------------


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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  //-----------------------------stepper------------------------------------------

  Stepper_Disable();
  HAL_Delay(1000);

  Stepper_Enable();


  //---------------------------------------------------------------------------------

  //-----------------------------stepper------------------------------------------

  Encoder_Start();

  //---------------------------------------------------------------------------------

  //------------------Maquina estados Homing-------------------------------------------

  homing_state = HOMING_START;
  homing_done = 0;


  //---------------------------------------------------------------------------------


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Homing_Update();
/*
	  limit_min_hit = 0;
	     limit_max_hit = 0;

	     // Mover hacia un lado
	     Stepper_StartMmS(40.0f, 0);

	     while (!limit_min_hit)
	     {
	         // seguridad extra por polling
	         if (LimitMin_IsPressed())
	         {
	             limit_min_hit = 1;
	             Stepper_StopHold();
	             break;
	         }
	     }

	     Stepper_StopHold();
	     HAL_Delay(1000);

	     limit_min_hit = 0;
	     limit_max_hit = 0;

	     // Mover hacia el otro lado
	     Stepper_StartMmS(40.0f, 1);

	     while (!limit_max_hit)
	     {
	         // seguridad extra por polling
	         if (LimitMax_IsPressed())
	         {
	             limit_max_hit = 1;
	             Stepper_StopHold();
	             break;
	         }
	     }

	     Stepper_StopHold();
	     HAL_Delay(1000);

*/
//------------------------------------------------
/*

	  // Sentido 1: 20 mm/s durante 3 segundos
	     Stepper_StartMmS(40.0f, 1);
	     HAL_Delay(2000);

	     Stepper_StopHold();
	     HAL_Delay(1000);

	     // Sentido 2: 20 mm/s durante 3 segundos
	     Stepper_StartMmS(40.0f, 0);
	     HAL_Delay(2000);

	     Stepper_StopHold();
	     HAL_Delay(1000);
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 49;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_Pin|DIR_Pin|ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_Pin DIR_Pin ENABLE_Pin */
  GPIO_InitStruct.Pin = EN_Pin|DIR_Pin|ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FC_Izquierda_Pin FC_Derecha_Pin */
  GPIO_InitStruct.Pin = FC_Izquierda_Pin|FC_Derecha_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
