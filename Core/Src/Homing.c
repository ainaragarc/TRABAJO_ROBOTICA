#include "Homing.h"
#include "EncoderRobot.h"
#include "FinalDeCarrera.h"
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#define PWM_PARADO          1500u
#define PWM_T_HOME          1600u
#define PWM_T_RETROCESO     1400u
#define PWM_I_HOME          1400u
#define PWM_I_RETROCESO     1600u
#define HOMING_TIMEOUT_MS   10000u
#define HOMING_BACKOFF_MM   5.0f

static HomingEstado estado   = HOMING_IDLE;
static uint32_t     t_inicio = 0;

extern EncoderRobot   encIzq;
extern EncoderRobot   encDer;
extern FinalDeCarrera limiteTraslacion;
extern FinalDeCarrera limiteInclinacion;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;

static void mover_traslacion(uint32_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm);
}

static void mover_inclinacion(uint32_t pwm) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
}

static void parar_todo(void) {
    mover_traslacion(PWM_PARADO);
    mover_inclinacion(PWM_PARADO);
}

void Homing_Iniciar(void) {
    estado   = HOMING_TRASLACION;
    t_inicio = HAL_GetTick();
    mover_traslacion(PWM_T_HOME);
    printf("HOMING: traslacion iniciada\r\n");
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
        case HOMING_COMPLETO:
        case HOMING_ERROR:
            return;

        case HOMING_TRASLACION:
            if (HAL_GetTick() - t_inicio > HOMING_TIMEOUT_MS) {
                parar_todo();
                estado = HOMING_ERROR;
                printf("HOMING ERROR: timeout traslacion\r\n");
                break;
            }
            if (FinalDeCarrera_getFlag(&limiteTraslacion)) {
                FinalDeCarrera_resetFlag(&limiteTraslacion);
                mover_traslacion(PWM_PARADO);
                EncoderRobot_reset(&encIzq);
                mover_traslacion(PWM_T_RETROCESO);
                t_inicio = HAL_GetTick();
                estado   = HOMING_RETROCESO_T;
                printf("HOMING: traslacion en limite, retrocediendo\r\n");
            }
            break;

        case HOMING_RETROCESO_T:
            if (fabsf(EncoderRobot_getDistanciaMM(&encIzq)) >= HOMING_BACKOFF_MM) {
                mover_traslacion(PWM_PARADO);
                EncoderRobot_reset(&encIzq);
                mover_inclinacion(PWM_I_HOME);
                t_inicio = HAL_GetTick();
                estado   = HOMING_INCLINACION;
                printf("HOMING: inclinacion iniciada\r\n");
            }
            break;

        case HOMING_INCLINACION:
            if (HAL_GetTick() - t_inicio > HOMING_TIMEOUT_MS) {
                parar_todo();
                estado = HOMING_ERROR;
                printf("HOMING ERROR: timeout inclinacion\r\n");
                break;
            }
            if (FinalDeCarrera_getFlag(&limiteInclinacion)) {
                FinalDeCarrera_resetFlag(&limiteInclinacion);
                mover_inclinacion(PWM_PARADO);
                EncoderRobot_reset(&encDer);
                mover_inclinacion(PWM_I_RETROCESO);
                t_inicio = HAL_GetTick();
                estado   = HOMING_RETROCESO_I;
                printf("HOMING: inclinacion en limite, retrocediendo\r\n");
            }
            break;

        case HOMING_RETROCESO_I:
            if (fabsf(EncoderRobot_getDistanciaMM(&encDer)) >= HOMING_BACKOFF_MM) {
                mover_inclinacion(PWM_PARADO);
                EncoderRobot_reset(&encDer);
                estado = HOMING_COMPLETO;
                printf("HOMING: COMPLETO. Robot listo.\r\n");
            }
            break;
    }
}
