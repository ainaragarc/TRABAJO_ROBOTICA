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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "motores.h"
#include "EncoderRobot.h"
#include "FinalDeCarrera.h"
#include "Homing.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
EncoderRobot encIzq;             // traslación corredera (TIM2)
EncoderRobot encDer;             // inclinación primer eslabón (TIM3)
FinalDeCarrera limiteIzq;        // PA10
FinalDeCarrera limiteDer;        // PA12
FinalDeCarrera limiteInclinacion;// PA11
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

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // CAMBIO: htim1->htim2/htim3, PPR 600->2400 (cuadratura x4)
  EncoderRobot_init(&encIzq, &htim2, 2400, 65.0f);
  EncoderRobot_init(&encDer, &htim3, 2400, 65.0f);

  FinalDeCarrera_init(&limiteIzq,         GPIOA, GPIO_PIN_10, false);
  FinalDeCarrera_init(&limiteInclinacion, GPIOA, GPIO_PIN_11, false);
  FinalDeCarrera_init(&limiteDer,         GPIOA, GPIO_PIN_12, false);

  Robot_InitMotores();
  Robot_InitEncoders();

  // Test motor inclinación — velocidades corregidas a rango válido (-100 a 100)
  motor1_mover_grados_estimados(&htim5, 90.0f, 50);
  HAL_Delay(2000);
  motor1_mover_grados_estimados(&htim5, -90.0f, 50);
  HAL_Delay(2000);
  motor1_set_velocidad(&htim5, -50);
  HAL_Delay(2000);
  motor1_set_velocidad(&htim5, 0);
  HAL_Delay(1000);
  motor1_set_velocidad(&htim5, 50);
  HAL_Delay(2000);
  motor1_set_velocidad(&htim5, 0);
  /* USER CODE END 2 */

  while (1)
  {
    Robot_Tick();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 8;
  RCC_OscInitStruct.PLL.PLLN            = 100;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance               = TIM1;
  htim1.Init.Prescaler         = 99;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.Period            = 19999;
  htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = 0;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 4294967295;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 4;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 4;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance               = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = 65535;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter    = 4;
  sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter    = 4;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM5_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim5.Instance               = TIM5;
  htim5.Init.Prescaler         = 49;
  htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim5.Init.Period            = 19999;
  htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) Error_Handler();

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) Error_Handler();

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();

  HAL_TIM_MspPostInit(&htim5);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOD, STEP_PAP_Pin | DIR_PAP_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin   = STEP_PAP_Pin | DIR_PAP_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  // CAMBIO: eliminada doble inicialización, solo RISING_FALLING con PULLUP
  GPIO_InitStruct.Pin  = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

void Robot_InitMotores(void) {
    // CAMBIO: eliminadas referencias a htim1, todo en htim5
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);  // motor1 inclinación
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);  // servo codo
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);  // servo muñeca

    motor1_set_velocidad(&htim5, 0);
    set_servo_2(&htim5, entero_pos(90.0f));
    set_servo_3(&htim5, entero_pos(0.0f));
}

void Robot_InitEncoders(void) {
    EncoderRobot_inicializar(&encIzq);
    EncoderRobot_inicializar(&encDer);
    stepper_reset_posicion();  // CAMBIO: añadido reset posición stepper
    Homing_Iniciar();
}

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
        distAntIzq = telemetria.distIzq;
        distAntDer = telemetria.distDer;
    }
}

void Robot_VerificarLimites(void) {
    if (FinalDeCarrera_getFlag(&limiteIzq)) {
        stepper_iniciar_movimiento(0, 0, 1);   // parar stepper
        peligroObstaculo = true;
        EncoderRobot_reset(&encIzq);
        stepper_reset_posicion();              // CAMBIO: reset posición absoluta
        FinalDeCarrera_resetFlag(&limiteIzq);
    }
    if (FinalDeCarrera_getFlag(&limiteDer)) {
        stepper_iniciar_movimiento(0, 0, 1);
        peligroObstaculo = true;
        FinalDeCarrera_resetFlag(&limiteDer);
    }
    if (FinalDeCarrera_getFlag(&limiteInclinacion)) {
        motor1_parar();                        // CAMBIO: htim1 → motor1_parar()
        peligroObstaculo = true;
        EncoderRobot_reset(&encDer);
        FinalDeCarrera_resetFlag(&limiteInclinacion);
    }
}

void Robot_DebugSistema(void) {
    // debug desactivado — CAMBIO: eliminada llave suelta que causaba error
}

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

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
