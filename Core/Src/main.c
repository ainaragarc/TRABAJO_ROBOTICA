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
// Prescaler TIM4 = 49 -> tick = 1 MHz -> 1 us.
#define STEP_TIMER_TICK_HZ      1000000UL

// Full-step
#define MICROSTEP               1.0f
#define FULL_STEPS_PER_REV      200.0f
#define STEPS_PER_REV           (FULL_STEPS_PER_REV * MICROSTEP)

// Corredera: 8 mm por vuelta
#define MM_PER_REV              8.0f
#define STEPS_PER_MM            (STEPS_PER_REV / MM_PER_REV)

// Velocidad elegida
#define BASE_SPEED_MM_S         40.0f
#define MIN_SPEED_MM_S          10.0f

volatile uint8_t stepper_is_running = 0;
volatile uint8_t stepper_current_dir = 0;

#define DIR_TO_RIGHT            0
#define DIR_TO_LEFT             1

//-----------------------------------------------------------------------


//------------------FCs-------------------------------------------

// PA9 = final derecho
// PA8 = final izquierdo

#define LIMIT_RIGHT_PORT        GPIOA
#define LIMIT_RIGHT_PIN         GPIO_PIN_9

#define LIMIT_LEFT_PORT         GPIOA
#define LIMIT_LEFT_PIN          GPIO_PIN_8

volatile uint8_t limit_right_hit = 0;
volatile uint8_t limit_left_hit = 0;

volatile uint32_t last_limit_right_ms = 0;
volatile uint32_t last_limit_left_ms = 0;
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

volatile uint8_t dbg_limit_right = 0;
volatile uint8_t dbg_limit_left = 0;

//-----------------------------------------------------------------------

//------------------Maquina estados Homing-------------------------------------------
typedef enum
{
    HOMING_START = 0,
    HOMING_MOVE_TO_RIGHT,
    HOMING_AT_RIGHT,
    HOMING_MOVE_TO_LEFT,
    HOMING_AT_LEFT,
    HOMING_MOVE_TO_MID,
    HOMING_DONE
} HomingState;


volatile HomingState homing_state = HOMING_START;

// Margen de parada en cuentas de encoder para el punto medio.
#define MID_TOLERANCE_COUNTS    10

//-----------------------------------------------------------------------

//------------------Motor M1 + FC-------------------------------------------
#define M1_SERVO_TIM            htim1
#define M1_SERVO_CHANNEL        TIM_CHANNEL_1   // Cambiar a CH2/CH3 si tu señal está en otro pin

// Para esta prueba uso PA8 como final de cero.
// Si el final de motor 1 está en otro pin, cambia estas dos líneas.
#define M1_ZERO_FC_PORT         GPIOA
#define M1_ZERO_FC_PIN          GPIO_PIN_10

// Servo continuo: ajustar si tu servo no se queda quieto exactamente en 1500
#define M1_SERVO_STOP_US        1500

// Velocidad lenta hacia el final.
// Si se mueve en sentido contrario al final, cambia 1420 por 1580.
#define M1_SERVO_HOME_US        1200

#define M1_SERVO_MIN_US         1000
#define M1_SERVO_MAX_US         2000

// Seguridad: si en 3 s no encuentra el final, para.
#define M1_HOME_TIMEOUT_MS      1500

// Antirrebote del final
#define M1_FC_DEBOUNCE_MS       30

typedef enum
{
    M1_IDLE = 0,
    M1_HOMING,
    M1_ZERO_OK,
    M1_ERROR_TIMEOUT
} Motor1State;

volatile Motor1State m1_state = M1_IDLE;

volatile uint8_t m1_zero_done = 0;
volatile uint8_t m1_error = 0;

volatile uint16_t dbg_m1_pwm_us = 1500;
volatile uint8_t dbg_m1_fc_zero = 0;
volatile uint8_t dbg_m1_state = 0;

static uint32_t m1_home_start_ms = 0;
static uint32_t m1_fc_pressed_since_ms = 0;

//-----------------------------------------------------------------------

//------------------------------Encoder Motor 1-----------------------------------------
#define M1_ENC_TIM              htim2

volatile int32_t m1_encoder_counts = 0;
volatile int32_t m1_encoder_zero_counts = 0;

volatile int32_t dbg_m1_enc_counts = 0;
volatile uint32_t dbg_m1_enc_raw = 0;

//-----------------------------------------------------------------------

//------------------ Control posicion Motor 1 ----------------------------

// IMPORTANTE:
// Este valor hay que calibrarlo.
// Después de hacer homing, mueve el eje hasta 180 grados físicos
// y mira dbg_m1_enc_counts. Ese valor será M1_COUNTS_180.
#define M1_COUNTS_180              1000.0f   // provisional, luego se cambia

#define M1_CONTROL_PERIOD_MS       10

#define M1_POS_TOLERANCE_DEG       1.0f

// PWM alrededor del neutro
#define M1_PWM_MIN_MOVE_US         80.0f
#define M1_PWM_MAX_DELTA_US        400.0f

// Ganancia proporcional: us de PWM por grado de error
#define M1_KP_US_PER_DEG           8.0f

// Si al mandar +30 grados se mueve hacia el final PA10, cambia esto a -1
#define M1_CONTROL_SIGN            1

volatile float m1_target_deg = 0.0f;
volatile float m1_current_deg = 0.0f;
volatile float m1_error_deg = 0.0f;

volatile uint8_t m1_position_control_enabled = 0;

volatile uint16_t dbg_m1_control_pwm = 1500;
volatile float dbg_m1_target_deg = 0.0f;
volatile float dbg_m1_current_deg = 0.0f;
volatile float dbg_m1_error_deg = 0.0f;

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

// Prototipos Motor 1 - Encoder
static void Motor1_Encoder_Start(void);
static void Motor1_Encoder_ResetZero(void);
static void Motor1_Encoder_Update(void);

// Prototipos Motor 1 - Servo / Homing
static void Motor1_Servo_SetUs(uint16_t us);
static void Motor1_Servo_Stop(void);
static void Motor1_Servo_StartPWM(void);
static uint8_t Motor1_ZeroFC_IsPressedRaw(void);
static uint8_t Motor1_ZeroFC_IsPressedDebounced(void);
static void Motor1_HomeZero_Start(void);
static void Motor1_HomeZero_Update(void);

//prototipos control
static float Motor1_Counts_To_Deg(int32_t counts);
static int32_t Motor1_Deg_To_Counts(float deg);
static void Motor1_SetTargetDeg(float deg);
static void Motor1_PositionControl_Update(void);

//-----------------------------stepper------------------------------------------

static void Stepper_Enable(void)
{
    // A4988 ENABLE activo en LOW
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

    // Deja el A4988 activo para mantener par
    Stepper_Enable();
}

//-----------------------------------------------------------------------

//-----------------------------FCs------------------------------------------
static uint8_t LimitRight_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_RIGHT_PORT, LIMIT_RIGHT_PIN) == GPIO_PIN_RESET);
}

static uint8_t LimitLeft_IsPressed(void)
{
    return (HAL_GPIO_ReadPin(LIMIT_LEFT_PORT, LIMIT_LEFT_PIN) == GPIO_PIN_RESET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == LIMIT_RIGHT_PIN)
    {
        if (now - last_limit_right_ms < 50)
        {
            return;
        }

        last_limit_right_ms = now;

        if (LimitRight_IsPressed())
        {
            limit_right_hit = 1;

            if (stepper_is_running && stepper_current_dir == DIR_TO_RIGHT)
            {
                Stepper_StopHold();
            }
        }
    }

    if (GPIO_Pin == LIMIT_LEFT_PIN)
    {
        if (now - last_limit_left_ms < 50)
        {
            return;
        }

        last_limit_left_ms = now;

        if (LimitLeft_IsPressed())
        {
            limit_left_hit = 1;

            if (stepper_is_running && stepper_current_dir == DIR_TO_LEFT)
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
       TIM3 es de 16 bits. Esta resta con int16_t gestiona overflow.
       Si pasa de 65535 a 0, delta = +1.
       Si pasa de 0 a 65535, delta = -1.
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
static int32_t abs32(int32_t x)
{
    if (x < 0)
    {
        return -x;
    }
    return x;
}


static void Homing_Update(void)
{
    Encoder_Update();

    dbg_homing_state = (uint8_t)homing_state;
    dbg_limit_right = LimitRight_IsPressed();
    dbg_limit_left = LimitLeft_IsPressed();

    switch (homing_state)
    {
        case HOMING_START:
        {
            homing_done = 0;

            limit_right_hit = 0;
            limit_left_hit = 0;

            /*
               Primer movimiento:
               ir a la derecha hasta PA9.
            */
            Stepper_StartMmS(BASE_SPEED_MM_S, DIR_TO_RIGHT);

            homing_state = HOMING_MOVE_TO_RIGHT;
            break;
        }

        case HOMING_MOVE_TO_RIGHT:
        {
            if (LimitRight_IsPressed() || limit_right_hit)
            {
                Stepper_StopHold();
                HAL_Delay(200);

                /*
                   PA9, final derecho, pasa a ser posición cero.
                */
                Encoder_ResetPosition();

                limit_right_hit = 0;
                limit_left_hit = 0;

                homing_state = HOMING_AT_RIGHT;
            }
            break;
        }

        case HOMING_AT_RIGHT:
        {
            HAL_Delay(300);

            /*
               Desde la derecha vamos hacia la izquierda.
               Al ir a la izquierda, el encoder decrementa.
            */
            Stepper_StartMmS(BASE_SPEED_MM_S, DIR_TO_LEFT);

            homing_state = HOMING_MOVE_TO_LEFT;
            break;
        }

        case HOMING_MOVE_TO_LEFT:
        {
            if (LimitLeft_IsPressed() || limit_left_hit)
            {
                Stepper_StopHold();
                HAL_Delay(200);

                Encoder_Update();

                /*
                   Estamos en el final izquierdo.
                   Derecha era 0.
                   Izquierda puede ser negativa o positiva según el signo real del encoder.
                */
                encoder_total_counts = encoder_position_counts;
                encoder_mid_counts = encoder_total_counts / 2;

                /*
                   Seguridad:
                   Si el encoder apenas ha contado, algo va mal.
                   Pero ahora miramos valor absoluto, no signo.
                */
                if (abs32(encoder_total_counts) < 100)
                {
                    Stepper_StopHold();
                    homing_done = 0;
                    homing_state = HOMING_DONE;
                    break;
                }

                limit_right_hit = 0;
                limit_left_hit = 0;

                homing_state = HOMING_AT_LEFT;
            }
            break;
        }

        case HOMING_AT_LEFT:
        {
            HAL_Delay(300);

            /*
               Estamos a la izquierda, con encoder negativo.
               Volvemos hacia la derecha.
               Hacia la derecha el encoder incrementa.
            */
            Stepper_StartMmS(BASE_SPEED_MM_S, DIR_TO_RIGHT);

            homing_state = HOMING_MOVE_TO_MID;
            break;
        }

        case HOMING_MOVE_TO_MID:
        {
            /*
               Queremos ir desde el final izquierdo hacia el punto medio.

               Caso A:
               derecha = 0
               izquierda = -20000
               centro = -10000
               al ir hacia la derecha: -20000 -> -19000 -> ... -> -10000
               condición: encoder_position_counts >= encoder_mid_counts

               Caso B:
               derecha = 0
               izquierda = +20000
               centro = +10000
               al ir hacia la derecha: +20000 -> +19000 -> ... -> +10000
               condición: encoder_position_counts <= encoder_mid_counts
            */

            if (encoder_total_counts < 0)
            {
                if (encoder_position_counts >= encoder_mid_counts - MID_TOLERANCE_COUNTS)
                {
                    Stepper_StopHold();

                    homing_done = 1;
                    homing_state = HOMING_DONE;
                }
            }
            else
            {
                if (encoder_position_counts <= encoder_mid_counts + MID_TOLERANCE_COUNTS)
                {
                    Stepper_StopHold();

                    homing_done = 1;
                    homing_state = HOMING_DONE;
                }
            }

            /*
               Seguridad:
               Si toca el final derecho antes de llegar al centro, paramos.
            */
            if (LimitRight_IsPressed() || limit_right_hit)
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
            homing_done = 0;
            homing_state = HOMING_DONE;
            break;
        }
    }
}

//-----------------------------------------------------------------------

//------------------ Motor 1: servo continuo + FC cero ------------------

static void Motor1_Servo_SetUs(uint16_t us)
{
    if (us < M1_SERVO_MIN_US)
    {
        us = M1_SERVO_MIN_US;
    }

    if (us > M1_SERVO_MAX_US)
    {
        us = M1_SERVO_MAX_US;
    }

    __HAL_TIM_SET_COMPARE(&M1_SERVO_TIM, M1_SERVO_CHANNEL, us);

    dbg_m1_pwm_us = us;
}

static void Motor1_Servo_Stop(void)
{
    Motor1_Servo_SetUs(M1_SERVO_STOP_US);
}

static void Motor1_Servo_StartPWM(void)
{
    HAL_TIM_PWM_Start(&M1_SERVO_TIM, M1_SERVO_CHANNEL);

    // Muy importante: al arrancar, servo parado.
    Motor1_Servo_Stop();
}

static uint8_t Motor1_ZeroFC_IsPressedRaw(void)
{
    // Final con pull-up: pulsado = LOW
    return (HAL_GPIO_ReadPin(M1_ZERO_FC_PORT, M1_ZERO_FC_PIN) == GPIO_PIN_RESET);
}

static uint8_t Motor1_ZeroFC_IsPressedDebounced(void)
{
    uint32_t now = HAL_GetTick();

    if (Motor1_ZeroFC_IsPressedRaw())
    {
        if (m1_fc_pressed_since_ms == 0)
        {
            m1_fc_pressed_since_ms = now;
        }

        if ((now - m1_fc_pressed_since_ms) >= M1_FC_DEBOUNCE_MS)
        {
            return 1;
        }
    }
    else
    {
        m1_fc_pressed_since_ms = 0;
    }

    return 0;
}

static void Motor1_HomeZero_Start(void)
{
	 m1_zero_done = 0;
	    m1_error = 0;
	    m1_fc_pressed_since_ms = 0;

	    // Por seguridad, antes de empezar, servo parado
	    Motor1_Servo_Stop();

	    // Si al arrancar ya está pisando el final PA10,
	    // esa posición es directamente el cero físico.
	    if (Motor1_ZeroFC_IsPressedRaw())
	    {
	        Motor1_Servo_Stop();

	        // Seteamos encoder a cero en la posición del final
	        Motor1_Encoder_ResetZero();

	        m1_zero_done = 1;
	        m1_error = 0;
	        m1_state = M1_ZERO_OK;

	        // Activamos control manteniendo posición 0 grados
	        Motor1_SetTargetDeg(0.0f);

	        return;
	    }

	    // Si no está en el final, empezamos búsqueda de cero
	    m1_home_start_ms = HAL_GetTick();

	    // Movimiento hacia el final de carrera PA10
	    Motor1_Servo_SetUs(M1_SERVO_HOME_US);

	    m1_state = M1_HOMING;
}

static void Motor1_HomeZero_Update(void)
{
    uint32_t now = HAL_GetTick();

    dbg_m1_fc_zero = Motor1_ZeroFC_IsPressedRaw();
    dbg_m1_state = (uint8_t)m1_state;

    switch (m1_state)
    {
        case M1_IDLE:
        {
            Motor1_Servo_Stop();
            break;
        }

        case M1_HOMING:
        {
        	if (Motor1_ZeroFC_IsPressedDebounced())
        	{
        		Motor1_Servo_Stop();

        		    Motor1_Encoder_ResetZero();

        		    m1_zero_done = 1;
        		    m1_error = 0;
        		    m1_state = M1_ZERO_OK;

        		    // Mantener cero después del homing
        		    Motor1_SetTargetDeg(0.0f);
        	    break;
        	}

            if ((now - m1_home_start_ms) > M1_HOME_TIMEOUT_MS)
            {
                Motor1_Servo_Stop();

                m1_zero_done = 0;
                m1_error = 1;
                m1_state = M1_ERROR_TIMEOUT;
                break;
            }

            break;
        }

        case M1_ZERO_OK:
        {

            break;
        }

        case M1_ERROR_TIMEOUT:
        {
            Motor1_Servo_Stop();
            break;
        }

        default:
        {
            Motor1_Servo_Stop();

            m1_zero_done = 0;
            m1_error = 1;
            m1_state = M1_ERROR_TIMEOUT;
            break;
        }
    }
}

//-----------------------------------------------------------------------

//------------------ Encoder Motor 1 ------------------------------------

static void Motor1_Encoder_Start(void)
{
    HAL_TIM_Encoder_Start(&M1_ENC_TIM, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&M1_ENC_TIM, 0);

    m1_encoder_counts = 0;
    m1_encoder_zero_counts = 0;

    dbg_m1_enc_counts = 0;
    dbg_m1_enc_raw = 0;
}

static void Motor1_Encoder_ResetZero(void)
{
    __HAL_TIM_SET_COUNTER(&M1_ENC_TIM, 0);

    m1_encoder_counts = 0;
    m1_encoder_zero_counts = 0;

    dbg_m1_enc_counts = 0;
    dbg_m1_enc_raw = 0;
}

static void Motor1_Encoder_Update(void)
{
    uint32_t raw = __HAL_TIM_GET_COUNTER(&M1_ENC_TIM);

    /*
       TIM2 es de 32 bits.

       Si el encoder va en sentido positivo:
       raw = 0, 1, 2, 3...

       Si va en sentido negativo desde cero:
       raw = 0xFFFFFFFF, 0xFFFFFFFE...
       Al convertirlo a int32_t se convierte en -1, -2...
    */
    m1_encoder_counts = (int32_t)raw;

    dbg_m1_enc_raw = raw;
    dbg_m1_enc_counts = m1_encoder_counts;
}

//-----------------------------------------------------------------------

//------------------ Control posicion Motor 1 ----------------------------

static float Motor1_Counts_To_Deg(int32_t counts)
{
    return ((float)counts * 180.0f) / M1_COUNTS_180;
}

static int32_t Motor1_Deg_To_Counts(float deg)
{
    return (int32_t)((deg * M1_COUNTS_180) / 180.0f);
}

static void Motor1_SetTargetDeg(float deg)
{
    if (deg < 0.0f)
    {
        deg = 0.0f;
    }

    if (deg > 180.0f)
    {
        deg = 180.0f;
    }

    m1_target_deg = deg;
    m1_position_control_enabled = 1;

    dbg_m1_target_deg = m1_target_deg;
}

static void Motor1_PositionControl_Update(void)
{
    static uint32_t last_control_ms = 0;

    uint32_t now = HAL_GetTick();

    if ((now - last_control_ms) < M1_CONTROL_PERIOD_MS)
    {
        return;
    }

    last_control_ms = now;

    if (m1_state == M1_HOMING)
    {
        return;
    }

    // Si no se ha hecho cero todavía, no dejamos controlar posición
    if (!m1_zero_done || m1_error)
    {
        Motor1_Servo_Stop();
        m1_position_control_enabled = 0;
        return;
    }

    if (!m1_position_control_enabled)
    {
        Motor1_Servo_Stop();
        return;
    }

    Motor1_Encoder_Update();

    m1_current_deg = Motor1_Counts_To_Deg(m1_encoder_counts);
    m1_error_deg = m1_target_deg - m1_current_deg;

    dbg_m1_current_deg = m1_current_deg;
    dbg_m1_error_deg = m1_error_deg;
    dbg_m1_target_deg = m1_target_deg;

    // Seguridad de cero:
    // si toca el final PA10, esa posición vuelve a ser cero absoluto.
    if (Motor1_ZeroFC_IsPressedRaw())
    {
        Motor1_Encoder_ResetZero();
        m1_current_deg = 0.0f;

        // Si el objetivo era cerca de cero, nos quedamos parados.
        if (m1_target_deg <= 1.0f)
        {
            Motor1_Servo_Stop();
            dbg_m1_control_pwm = M1_SERVO_STOP_US;
            return;
        }
    }

    // Zona muerta: suficientemente cerca del objetivo
    if (fabsf(m1_error_deg) <= M1_POS_TOLERANCE_DEG)
    {
        Motor1_Servo_Stop();
        dbg_m1_control_pwm = M1_SERVO_STOP_US;
        return;
    }

    float abs_error = fabsf(m1_error_deg);

    float delta_pwm = M1_KP_US_PER_DEG * abs_error;

    if (delta_pwm < M1_PWM_MIN_MOVE_US)
    {
        delta_pwm = M1_PWM_MIN_MOVE_US;
    }

    if (delta_pwm > M1_PWM_MAX_DELTA_US)
    {
        delta_pwm = M1_PWM_MAX_DELTA_US;
    }

    int16_t pwm_cmd = M1_SERVO_STOP_US;

    if (m1_error_deg > 0.0f)
    {
        pwm_cmd = M1_SERVO_STOP_US + (int16_t)(M1_CONTROL_SIGN * delta_pwm);
    }
    else
    {
        pwm_cmd = M1_SERVO_STOP_US - (int16_t)(M1_CONTROL_SIGN * delta_pwm);
    }

    // Seguridad extra:
    // si estamos intentando ir hacia negativo y el final está pulsado, paramos.
    if (Motor1_ZeroFC_IsPressedRaw() && pwm_cmd < M1_SERVO_STOP_US)
    {
        Motor1_Servo_Stop();
        Motor1_Encoder_ResetZero();
        dbg_m1_control_pwm = M1_SERVO_STOP_US;
        return;
    }

    Motor1_Servo_SetUs((uint16_t)pwm_cmd);
    dbg_m1_control_pwm = (uint16_t)pwm_cmd;
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

  //Stepper_Enable();


  //---------------------------------------------------------------------------------

  //-----------------------------encoder base------------------------------------------

  Encoder_Start();

  //---------------------------------------------------------------------------------

  //------------------Maquina estados Homing-------------------------------------------

  //homing_state = HOMING_START;
  //homing_done = 0;


  //---------------------------------------------------------------------------------

  //------------------Motor M1-------------------------------------------

  Motor1_Servo_StartPWM();
  Motor1_Encoder_Start();
  HAL_Delay(1000);
  Motor1_HomeZero_Start();

  //---------------------------------------------------------------------------------

  //-----------------------------encoder motor 1------------------------------------------



  //---------------------------------------------------------------------------------

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //Homing_Update();

	  // 1) Primero homing
	      Motor1_HomeZero_Update();

	      // 2) Solo hacemos control de posición cuando el cero ya está hecho
	      if (m1_zero_done && !m1_error)
	      {
	          Motor1_PositionControl_Update();
	      }

	      // 3) Prueba temporal: mandar 30 grados después del homing
	      static uint8_t test_sent = 0;
	      static uint32_t zero_ok_time = 0;

	      if (m1_zero_done && !m1_error && test_sent == 0)
	      {
	          if (zero_ok_time == 0)
	          {
	              zero_ok_time = HAL_GetTick();
	          }

	          if ((HAL_GetTick() - zero_ok_time) > 1000)
	          {
	              Motor1_SetTargetDeg(30.0f);
	              test_sent = 1;
	          }
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
  sConfig.IC1Filter = 8;
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

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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
