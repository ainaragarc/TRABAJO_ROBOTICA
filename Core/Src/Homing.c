#include "Homing.h"
#include "EncoderRobot.h"
#include "FinalDeCarrera.h"
#include "motores.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#define PWM_PARADO         1500u
#define PWM_T_HOME         1600u
#define PWM_T_RETROCESO    1400u
#define PWM_I_HOME         1400u
#define PWM_I_RETROCESO    1600u
#define HOMING_TIMEOUT_MS  10000u
#define HOMING_BACKOFF_MM  5.0f
#define HOMING_BACKOFF_GRADOS  5.0f

static HomingEstado estado   = HOMING_IDLE;
static uint32_t     t_inicio = 0;

extern EncoderRobot   encIzq;
extern EncoderRobot   encDer;
extern FinalDeCarrera limiteIzq;
extern FinalDeCarrera limiteDer;
extern FinalDeCarrera limiteInclinacion;
extern TIM_HandleTypeDef htim5;
// CAMBIO: eliminado extern htim1 — ya no se usa en ningún sitio

// La traslación en homing se hace con el stepper (parar = 0 pasos)
// y con PWM directo para el servo de traslación si lo hubiera
static void mover_traslacion_pwm(uint32_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm);
}

// CAMBIO: mover_inclinacion ahora usa TIM5_CH2 (motor1) en vez de htim1_CH1
static void mover_inclinacion(uint32_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm);
}

static void parar_todo(void) {
    mover_traslacion_pwm(PWM_PARADO);
    motor1_parar();  // CAMBIO: motor1_parar() en vez de htim1
}

void Homing_Iniciar(void) {
    // BYPASS temporal para testing — descomentar las líneas de abajo cuando homing esté listo
    estado = HOMING_COMPLETO;

    // estado   = HOMING_TRASLACION;
    // mover_traslacion_pwm(PWM_T_HOME);
    // mover_inclinacion_pwm(PWM_T_HOME);
    t_inicio = HAL_GetTick();
}

bool Homing_EstaCompleto(void) {
    return estado == HOMING_COMPLETO;
}

bool Homing_EstaActivo(void) {
    return (estado != HOMING_IDLE     &&
            estado != HOMING_COMPLETO &&
            estado != HOMING_ERROR);
}

HomingEstado Homing_GetEstado(void) {
    return estado;
}

void Homing_Tick(void) {
    switch (estado) {

        case HOMING_IDLE:
        	// Posible estado de espera para empezar el homing.
        case HOMING_COMPLETO:
        case HOMING_ERROR:
            return;

        case HOMING_TRASLACION:



            if (HAL_GetTick() - t_inicio > HOMING_TIMEOUT_MS) {
                parar_todo();
                estado = HOMING_ERROR; // Demasiado tiempo ha durado el homing
                break;
            }
            if (FinalDeCarrera_getFlag(&limiteIzq)) {
                FinalDeCarrera_resetFlag(&limiteIzq);
                mover_traslacion_pwm(PWM_PARADO);

                mover_traslacion_pwm(PWM_T_RETROCESO);
                t_inicio = HAL_GetTick();
                estado   = HOMING_RETROCESO_T;
            }
            break;

        case HOMING_RETROCESO_T:
            if (fabsf(EncoderRobot_getDistanciaMM(&encIzq)) >= HOMING_BACKOFF_MM) {
                mover_traslacion_pwm(PWM_PARADO);
                EncoderRobot_reset(&encIzq);

                stepper_reset_posicion();  // CAMBIO: fijar home stepper aquí
                mover_inclinacion(PWM_I_HOME);

                t_inicio = HAL_GetTick();
                estado   = HOMING_INCLINACION;
            }
            break;

        case HOMING_INCLINACION:
            if (HAL_GetTick() - t_inicio > HOMING_TIMEOUT_MS) {
                parar_todo();
                estado = HOMING_ERROR;
                break;
            }
            if (FinalDeCarrera_getFlag(&limiteInclinacion)) {
                FinalDeCarrera_resetFlag(&limiteInclinacion);
                motor1_parar();            // CAMBIO: motor1_parar() en vez de htim1

                mover_inclinacion(PWM_I_RETROCESO);
                t_inicio = HAL_GetTick();
                estado   = HOMING_RETROCESO_I;
            }
            break;

        case HOMING_RETROCESO_I:
            if (fabsf(EncoderRobot_getAnguloGrados(&encDer)) >= HOMING_BACKOFF_MM) {
                motor1_parar();            // CAMBIO: motor1_parar() en vez de htim1
                EncoderRobot_reset(&encDer);
                estado = HOMING_COMPLETO;
            }
            break;
    }
}
