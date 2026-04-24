#include "Homing.h"
#include "EncoderRobot.h"
#include "FinalDeCarrera.h"
#include <stdio.h>
#include <math.h>

// ── Parámetros ajustables ─────────────────────────────────────────────────────
// Si el motor va en sentido contrario, intercambia HOME y RETROCESO del eje.

#define PWM_PARADO          1500u

#define PWM_T_HOME          1600u   // Traslación → derecha (hacia fin de carrera)
#define PWM_T_RETROCESO     1400u   // Traslación → izquierda (alejarse)

#define PWM_I_HOME          1400u   // Inclinación → hacia el lienzo
#define PWM_I_RETROCESO     1600u   // Inclinación → alejarse del lienzo

#define HOMING_TIMEOUT_MS   10000u  // Aborta si un eje tarda más de 10 s
#define HOMING_BACKOFF_MM   5.0f    // mm que retrocede tras tocar el switch

// ── Estado interno ────────────────────────────────────────────────────────────

static HomingEstado estado  = HOMING_IDLE;
static uint32_t     t_inicio = 0;

// ── Recursos definidos en main.c ──────────────────────────────────────────────

extern EncoderRobot   encIzq;            // encoder traslación  (TIM2)
extern EncoderRobot   encDer;            // encoder inclinación (TIM3)
extern FinalDeCarrera limiteTraslacion;  // PA10 → fin de carrera traslación
extern FinalDeCarrera limiteInclinacion; // PA11 → fin de carrera inclinación
extern TIM_HandleTypeDef htim5;          // canal traslación en CH2
extern TIM_HandleTypeDef htim1;          // canal inclinación en CH1

// ── Helpers privados de motor ─────────────────────────────────────────────────

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

// ── API pública ───────────────────────────────────────────────────────────────

void Homing_Iniciar(void) {
    estado    = HOMING_TRASLACION;
    t_inicio  = HAL_GetTick();
    mover_traslacion(PWM_T_HOME);
    printf("HOMING: traslacion iniciada\r\n");
}

bool Homing_EstaCompleto(void) {
    return estado == HOMING_COMPLETO;
}

bool Homing_EstaActivo(void) {
    return (estado != HOMING_IDLE    &&
            estado != HOMING_COMPLETO &&
            estado != HOMING_ERROR);
}

HomingEstado Homing_GetEstado(void) {
    return estado;
}

// ── Máquina de estados ────────────────────────────────────────────────────────
//
// Se llama cada iteración del bucle principal (Robot_Tick).
// No bloquea: cada case comprueba una condición y transiciona si se cumple.

void Homing_Tick(void) {
    switch (estado) {

        case HOMING_IDLE:
        case HOMING_COMPLETO:
        case HOMING_ERROR:
            return;

        // ── 1. Mover hacia el fin de carrera de traslación ──────────────────
        case HOMING_TRASLACION:
            if (HAL_GetTick() - t_inicio > HOMING_TIMEOUT_MS) {
                parar_todo();
                estado = HOMING_ERROR;
                printf("HOMING ERROR: timeout traslacion\r\n");
                break;
            }
            if (limiteTraslacion.getFlag()) {
                limiteTraslacion.resetFlag();
                mover_traslacion(PWM_PARADO);
                encIzq.reset();                         // contador en 0 en el switch
                mover_traslacion(PWM_T_RETROCESO);
                t_inicio = HAL_GetTick();
                estado   = HOMING_RETROCESO_T;
                printf("HOMING: traslacion en limite, retrocediendo\r\n");
            }
            break;

        // ── 2. Retroceder hasta BACKOFF_MM y fijar el home real ─────────────
        case HOMING_RETROCESO_T:
            if (fabsf(encIzq.getDistanciaMM()) >= HOMING_BACKOFF_MM) {
                mover_traslacion(PWM_PARADO);
                encIzq.reset();                         // 0 mm = posición home útil
                mover_inclinacion(PWM_I_HOME);
                t_inicio = HAL_GetTick();
                estado   = HOMING_INCLINACION;
                printf("HOMING: inclinacion iniciada\r\n");
            }
            break;

        // ── 3. Mover hacia el fin de carrera de inclinación ─────────────────
        case HOMING_INCLINACION:
            if (HAL_GetTick() - t_inicio > HOMING_TIMEOUT_MS) {
                parar_todo();
                estado = HOMING_ERROR;
                printf("HOMING ERROR: timeout inclinacion\r\n");
                break;
            }
            if (limiteInclinacion.getFlag()) {
                limiteInclinacion.resetFlag();
                mover_inclinacion(PWM_PARADO);
                encDer.reset();
                mover_inclinacion(PWM_I_RETROCESO);
                t_inicio = HAL_GetTick();
                estado   = HOMING_RETROCESO_I;
                printf("HOMING: inclinacion en limite, retrocediendo\r\n");
            }
            break;

        // ── 4. Retroceder y fijar home de inclinación ────────────────────────
        case HOMING_RETROCESO_I:
            if (fabsf(encDer.getDistanciaMM()) >= HOMING_BACKOFF_MM) {
                mover_inclinacion(PWM_PARADO);
                encDer.reset();                         // 0 mm = posición home útil
                estado = HOMING_COMPLETO;
                printf("HOMING: COMPLETO. Robot listo.\r\n");
            }
            break;
    }
}
