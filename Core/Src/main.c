/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Programa de prueba progresiva del robot.
  *
  * Hardware confirmado (del .ioc):
  *  - PA0/PB3   TIM2 encoder izquierdo (rotación continua de inclinación)
  *  - PA6/PA7   TIM3 encoder derecho   (acoplado al husillo de traslación)
  *  - PA1       TIM5_CH2 servo continuo "base/inclinación"
  *  - PA2       TIM5_CH3 servo posicional "codo"
  *  - PA3       TIM5_CH4 servo posicional "muñeca"
  *  - PE13      TIM1_CH3 servo revólver
  *  - PD0/PD1   STEP/DIR stepper traslación (8 mm/vuelta, 200 pasos/vuelta)
  *  - PA8       fin de carrera inclinación (EXTI)
  *  - PA9, PA10 finales de carrera traslación (EXTI)
  *  - PA4, PA5  canal Z encoders (EXTI, opcional)
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
#include "movimiento.h"
#include "Homing.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANG_NEUTRO    90.0f     /* ángulo neutro de servos posicionales */
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
EncoderRobot encoderInclinacion;     /* Inclinación — TIM2, PA0/PB3          */
EncoderRobot encoderTraslacion;      /* Traslación  — TIM3, PA6/PA7          */

FinalDeCarrera limiteTraslacion_A;   /* PA9  — home corredera                */
FinalDeCarrera limiteTraslacion_B;   /* PA10 — fin de corredera              */
FinalDeCarrera limiteInclinacion;    /* PA8  — home inclinación              */

volatile bool peligroObstaculo = false;

typedef enum {
    MAIN_INIT = 0,
    MAIN_HOMING,
    MAIN_POSICION_FINAL,
    MAIN_LISTO,
    MAIN_ERROR,
} EstadoMain;

static EstadoMain estado_main = MAIN_INIT;

typedef struct { c3d punto; uint8_t flag; } Waypoint;

#define MAX_WAYPOINTS   20u
static Waypoint secuencia[MAX_WAYPOINTS];
static uint8_t  n_waypoints = 0;
static uint8_t  idx_wp      = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void Robot_InitPerifericos(void);
static void Robot_VerificarLimites(void);
static void parar_todo_motores(void);
static void test_rapido(void);
static void construir_secuencia(void);
static void Main_Tick(void);
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
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
    Robot_InitPerifericos();

    /* Posición segura inicial — servos a neutro, stepper parado */
    motor1_set_velocidad(&htim5, 0);
    set_servo_2(&htim5, entero_pos(ANG_NEUTRO));
    set_servo_3(&htim5, entero_pos(ANG_NEUTRO));
    set_servo_revolver(&htim1, 500u);
    HAL_Delay(1000);

    Homing_Iniciar();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        stepper_control_tick();   /* siempre — genera pulsos del stepper */
        Main_Tick();              /* ciclo de vida del robot             */
  /* USER CODE END 3 */
  }
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
  htim2.Init.Period = 65535;
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

  /*Configure GPIO pins : PA4 PA5 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_PAP_Pin DIR_PAP_Pin */
  GPIO_InitStruct.Pin = STEP_PAP_Pin|DIR_PAP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void Robot_InitPerifericos(void)
{
    EncoderRobot_init(&encoderInclinacion, &htim2, 4000, 65.0f);
    /* C1: usar init_lineal para el husillo — 8 mm/vuelta sin calcular diámetro virtual */
    EncoderRobot_init_lineal(&encoderTraslacion, &htim3, 4000, 8.0f);
    EncoderRobot_inicializar(&encoderInclinacion);
    EncoderRobot_inicializar(&encoderTraslacion);

    FinalDeCarrera_init(&limiteTraslacion_A, GPIOA, GPIO_PIN_9,  false);
    FinalDeCarrera_init(&limiteTraslacion_B, GPIOA, GPIO_PIN_10, false);
    FinalDeCarrera_init(&limiteInclinacion,  GPIOA, GPIO_PIN_8,  false);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);   /* PA1  inclinación */
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);   /* PA2  codo        */
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);   /* PA3  muñeca      */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);   /* PE13 revólver    */

    HAL_GPIO_WritePin(STEP_PAP_GPIO_Port, STEP_PAP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR_PAP_GPIO_Port,  DIR_PAP_Pin,  GPIO_PIN_RESET);
    stepper_reset_posicion();
}

static void parar_todo_motores(void)
{
    motor1_set_velocidad(&htim5, 0);
    pap_estado.pasos_restantes = 0;
}

/* Seguridad no bloqueante: peligroObstaculo dura 500 ms sin HAL_Delay */
static void Robot_VerificarLimites(void)
{
    static uint32_t t_peligro = 0;

    if (peligroObstaculo) {
        if (HAL_GetTick() - t_peligro >= 500u) peligroObstaculo = false;
        return;
    }

    bool tope = FinalDeCarrera_getFlag(&limiteTraslacion_A) ||
                FinalDeCarrera_getFlag(&limiteTraslacion_B) ||
                FinalDeCarrera_getFlag(&limiteInclinacion);

    if (tope) {
        peligroObstaculo = true;
        t_peligro = HAL_GetTick();
        parar_todo_motores();
        FinalDeCarrera_resetFlag(&limiteTraslacion_A);
        FinalDeCarrera_resetFlag(&limiteTraslacion_B);
        FinalDeCarrera_resetFlag(&limiteInclinacion);
    }
}

/* Test rápido de articulaciones — bloqueante, se ejecuta una sola vez */
static void test_rapido(void)
{
    set_servo_revolver(&htim1,  500u); HAL_Delay(500);
    set_servo_revolver(&htim1, 1500u); HAL_Delay(500);
    set_servo_revolver(&htim1, 2500u); HAL_Delay(500);
    set_servo_revolver(&htim1,  500u); HAL_Delay(500);

    set_servo_2(&htim5, entero_pos(60.0f));
    set_servo_3(&htim5, entero_pos(60.0f));  HAL_Delay(800);
    set_servo_2(&htim5, entero_pos(120.0f));
    set_servo_3(&htim5, entero_pos(120.0f)); HAL_Delay(800);
    set_servo_2(&htim5, entero_pos(ANG_NEUTRO));
    set_servo_3(&htim5, entero_pos(ANG_NEUTRO)); HAL_Delay(600);
}

/* Rellena secuencia[] con las rayas a pintar.
 * Lienzo 200×200 mm inclinado a 45°, 3 rayas horizontales.
 * Para añadir más rayas o cambiar posiciones, edita solo esta función. */
static void construir_secuencia(void)
{
    /* Alturas en el lienzo (mm) donde se pintará cada raya */
    static const int16_t alturas[] = { 50, 100, 150 };
    n_waypoints = 0;

    for (int i = 0; i < 3; i++) {
        c2d desde = { 0,   alturas[i] };   /* z=0 → inicio corredera  */
        c2d hasta = { 200, alturas[i] };   /* z=200 → fin del lienzo  */

        /* 1. Volar al inicio de la raya (sin contacto) */
        secuencia[n_waypoints++] = (Waypoint){ plano_retroceso(desde), 0 };
        /* 2. Apoyar el pincel en el lienzo */
        secuencia[n_waypoints++] = (Waypoint){ plano_dibujo(desde),    1 };
        /* 3. Trazar la raya de izquierda a derecha */
        secuencia[n_waypoints++] = (Waypoint){ plano_dibujo(hasta),    2 };
        /* 4. Retirar el pincel */
        secuencia[n_waypoints++] = (Waypoint){ plano_retroceso(hasta), 3 };
    }

    /* Posición de reposo al terminar */
    secuencia[n_waypoints++] = (Waypoint){ { SEPx, SEPy + 50, SEPz }, 0 };
}

/* ── Ciclo de vida: INIT → HOMING → TEST → SECUENCIA → LISTO ────────────── */
static void Main_Tick(void)
{
    switch (estado_main) {

    case MAIN_INIT:
        estado_main = MAIN_HOMING;
        break;

    case MAIN_HOMING:
        Homing_Tick();
        if (Homing_EstaCompleto()) {
            test_rapido();
            construir_secuencia();
            idx_wp = 0;
            velocidad_reset();
            estado_main = MAIN_POSICION_FINAL;
        } else if (Homing_GetEstado() == HOMING_ERROR) {
            parar_todo_motores();
            estado_main = MAIN_ERROR;
        }
        break;

    /* Ejecuta los waypoints de secuencia[] uno a uno */
    case MAIN_POSICION_FINAL: {
        Robot_VerificarLimites();
        if (idx_wp >= n_waypoints) {
            estado_main = MAIN_LISTO;
            break;
        }
        uint8_t  flag = secuencia[idx_wp].flag;
        motoresg sig;
        if (trayectoria_cutre(&sig, secuencia[idx_wp].punto, &flag)) {
            idx_wp++;
            velocidad_reset();
        } else {
            control_loop_motores(sig);
        }
        break;
    }

    case MAIN_LISTO:
        Robot_VerificarLimites();
        /* secuencia completada — robot en reposo */
        break;

    case MAIN_ERROR:
        parar_todo_motores();
        break;

    default:
        estado_main = MAIN_INIT;
        break;
    }
}

/* ── Callbacks de interrupciones GPIO ───────────────────────────────────── */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case GPIO_PIN_4:
        EncoderRobot_registrarVueltaZ(&encoderInclinacion);
        break;
    case GPIO_PIN_5:
        EncoderRobot_registrarVueltaZ(&encoderTraslacion);
        break;
    case GPIO_PIN_8:    /* PA8 — fin de carrera inclinación */
        FinalDeCarrera_onInterrupcion(&limiteInclinacion);
        break;
    case GPIO_PIN_9:    /* PA9 — fin de carrera traslación */
        FinalDeCarrera_onInterrupcion(&limiteTraslacion_A);
        break;
    case GPIO_PIN_10:   /* PA10 — fin de carrera traslación */
        FinalDeCarrera_onInterrupcion(&limiteTraslacion_B);
        break;
    default:
        break;
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
