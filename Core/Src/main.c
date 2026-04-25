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
#include <stdbool.h>

#include "motores.h"
#include "EncoderRobot.h"
#include "FinalDeCarrera.h"
#include "Homing.h"
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
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
// Encoder 1: Timer 2, 600 pulsos, rueda 65mm
EncoderRobot encIzq;

// Encoder 2: Timer 3, 600 pulsos, rueda 65mm
EncoderRobot encDer;


FinalDeCarrera limiteIzq;
FinalDeCarrera limiteDer;
FinalDeCarrera limiteInclinacion;
bool peligroObstaculo = false;
struct {
    float distIzq;
    float distDer;
    float angIzq;
    float angDer;
} telemetria;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Robot_InitMotores(void);
void Robot_InitEncoders(void);
void Robot_ActualizarTelemetria(void);
void Robot_TestEncoderManual(void);
void Robot_DebugSistema(void);
void Robot_VerificarLimites(void);
void Robot_Tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <math.h>


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
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  EncoderRobot_init(&encIzq, &htim1, 600, 65.0f);
  EncoderRobot_init(&encDer, &htim1, 600, 65.0f);
  FinalDeCarrera_init(&limiteIzq,          GPIOA, GPIO_PIN_10, false);
  FinalDeCarrera_init(&limiteDer,          GPIOA, GPIO_PIN_12, false);
  FinalDeCarrera_init(&limiteInclinacion,  GPIOA, GPIO_PIN_11, false);
  Robot_InitMotores();
  Robot_InitEncoders();

  motor1_mover_grados_estimados(&htim5, 90.0, 200);
  HAL_Delay(2000);

  motor1_mover_grados_estimados(&htim5, -90.0, 200);
  HAL_Delay(2000);

  motor1_set_velocidad(&htim5, -1000);
  HAL_Delay(2000);
  motor1_set_velocidad(&htim5, 0);
  HAL_Delay(1000);
  motor1_set_velocidad(&htim5, 1000);
  HAL_Delay(2000);
  motor1_set_velocidad(&htim5, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Robot_Tick();
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
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
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 49;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 19999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, STEP_PAP_Pin|DIR_PAP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP_PAP_Pin DIR_PAP_Pin */
  GPIO_InitStruct.Pin = STEP_PAP_Pin|DIR_PAP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Fines de carrera: PA10 (izquierdo) y PA11 (derecho)
  // Modo EXTI con PULLUP interno. El microswitch conecta a GND (contacto NO).
  GPIO_InitStruct.Pin  = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ── Inicialización ────────────────────────────────────────────────────────────

void Robot_InitMotores(void) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1500u);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, entero_pos(90.0f));
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, entero_pos(0.0f));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, entero_pos(45.0f));

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // Motor inclinación
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // Revólver

    // Secuencia de centrado mecánico (bloqueante, solo en arranque)
    float ang_codo_ini = 90.0f, ang_codo_fin = 45.0f;
    float ang_muneca_ini = 0.0f, ang_muneca_fin = 90.0f;
/*
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 2000u);
    HAL_Delay(300);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1500u);
    HAL_Delay(300);

    set_servo_2(&htim5, entero_pos(ang_codo_ini));
    set_servo_3(&htim5, entero_pos(ang_muneca_ini));
    set_servo_3(&htim5, entero_pos(ang_muneca_fin));
    set_servo_2(&htim5, entero_pos(ang_codo_fin));
    set_servo_2(&htim5, entero_pos(ang_codo_ini));
    set_servo_3(&htim5, entero_pos(ang_muneca_ini));

    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1500u);
    HAL_Delay(1000);
    */
}

void Robot_InitEncoders(void) {
    EncoderRobot_inicializar(&encIzq);
    EncoderRobot_inicializar(&encDer);
    Homing_Iniciar();
}

// ── Encoders ──────────────────────────────────────────────────────────────────

void Robot_ActualizarTelemetria(void) {
    telemetria.distIzq = EncoderRobot_getDistanciaMM(&encIzq);
    telemetria.distDer = EncoderRobot_getDistanciaMM(&encDer);
    telemetria.angIzq  = EncoderRobot_getAnguloGrados(&encIzq);
    telemetria.angDer  = EncoderRobot_getAnguloGrados(&encDer);
}

void Robot_TestEncoderManual(void) {
    static float distAntIzq = 0.0f, distAntDer = 0.0f;

    bool cambioIzq = fabsf(telemetria.distIzq - distAntIzq) > 1.0f;
    bool cambioDer = fabsf(telemetria.distDer - distAntDer) > 1.0f;

    if (cambioIzq || cambioDer) {
        //printf("IZQ -> Dist: %.2f mm | Ang: %.2f deg\r\n", telemetria.distIzq, telemetria.angIzq);
        //printf("DER -> Dist: %.2f mm | Ang: %.2f deg\r\n", telemetria.distDer, telemetria.angDer);
        distAntIzq = telemetria.distIzq;
        distAntDer = telemetria.distDer;
    }
}

// ── Seguridad ─────────────────────────────────────────────────────────────────

void Robot_VerificarLimites(void) {
    if (FinalDeCarrera_getFlag(&limiteIzq)) {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1500u);
        peligroObstaculo = true;
        EncoderRobot_reset(&encIzq);  // ← reset encoder traslación a 0
        FinalDeCarrera_resetFlag(&limiteIzq);
    }
    if (FinalDeCarrera_getFlag(&limiteDer)) {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 1500u);
        peligroObstaculo = true;
        FinalDeCarrera_resetFlag(&limiteDer);
    }
    if (FinalDeCarrera_getFlag(&limiteInclinacion)) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500u);
        peligroObstaculo = true;
        EncoderRobot_reset(&encDer);  // ← reset encoder inclinación a 0
        FinalDeCarrera_resetFlag(&limiteInclinacion);
    }
}

// ── Debug ─────────────────────────────────────────────────────────────────────

void Robot_DebugSistema(void) {
/*    if (Homing_EstaActivo()) {
        //static const char* nombres[] = {
        //    "IDLE","TRASLACION","RETROCESO_T",
        //    "INCLINACION","RETROCESO_I","COMPLETO","ERROR"
        };
        //printf("HOMING [%s]\r\n", nombres[Homing_GetEstado()]);
        return;
    }
    if (peligroObstaculo) {
        //printf("ALERTA! Final de carrera activado.\r\n");
    } else {
        //printf("IZQ:%.2f mm  DER:%.2f mm\r\n", telemetria.distIzq, telemetria.distDer);

  }*/
}
// ── Bucle principal ───────────────────────────────────────────────────────────

void Robot_Tick(void) {
    Homing_Tick();

    if (!Homing_EstaCompleto()) return;

    uint32_t tick = HAL_GetTick();

    stepper_control_tick();

    static uint32_t t_encoder = 0;
    if (tick - t_encoder >= 10) {
        t_encoder = tick;
        Robot_ActualizarTelemetria();
        Robot_TestEncoderManual();
    }

    static uint32_t t_debug = 0;
    if (tick - t_debug >= 100) {
        t_debug = tick;
        Robot_DebugSistema();
    }

    Robot_VerificarLimites();
}

// ── Interrupciones ────────────────────────────────────────────────────────────

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_4) {
        EncoderRobot_registrarVueltaZ(&encIzq);
    } else if (GPIO_Pin == GPIO_PIN_5) {
        EncoderRobot_registrarVueltaZ(&encDer);
    } else if (GPIO_Pin == GPIO_PIN_10) {
        FinalDeCarrera_onInterrupcion(&limiteIzq);
    } else if (GPIO_Pin == GPIO_PIN_11) {
        FinalDeCarrera_onInterrupcion(&limiteInclinacion);
    } else if (GPIO_Pin == GPIO_PIN_12) {
        FinalDeCarrera_onInterrupcion(&limiteDer);
    }

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
