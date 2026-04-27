#include "Homing.h"
#include "EncoderRobot.h"
#include "FinalDeCarrera.h"
#include "motores.h"
#include <math.h>

/* ── Constantes ──────────────────────────────────────────────────────────── */
#define VEL_R1_HOME         15      /* velocidad de búsqueda de PA8          */
#define TIMEOUT_R1_MS       10000u  /* máx tiempo por intento de dirección   */
#define BACKOFF_GRADOS      5.0f    /* separación del tope de inclinación    */
#define TIMEOUT_TRAS_MS     25000u  /* máx tiempo para cruzar la corredera   */
#define BACKOFF_MM          5.0f    /* separación del tope de traslación     */
#define VEL_STEPPER_HOMING  5u      /* intervalo stepper en homing (ms/paso) */

/* ── Externs ─────────────────────────────────────────────────────────────── */
extern EncoderRobot   encoderInclinacion;
extern EncoderRobot   encoderTraslacion;
extern FinalDeCarrera limiteTraslacion_A;
extern FinalDeCarrera limiteTraslacion_B;
extern FinalDeCarrera limiteInclinacion;
extern TIM_HandleTypeDef htim5;

/* ── Estado interno ──────────────────────────────────────────────────────── */
static HomingEstado estado    = HOMING_IDLE;
static uint32_t     t_inicio  = 0;
static int16_t      vel_r1    = VEL_R1_HOME;  /* signo = dirección probada  */
static uint8_t      dir_tras  = 0;            /* dirección home del stepper */

/* ── Helper: ángulo absoluto desde el último reset (sin módulo) ─────────── */
static float angulo_abs_r1(void) {
    return (float)EncoderRobot_getPulsosTotales(&encoderInclinacion)
           * (360.0f / (float)encoderInclinacion.ppr);
}

/* ── API pública ─────────────────────────────────────────────────────────── */
void Homing_Iniciar(void) {
    vel_r1   = VEL_R1_HOME;          /* primer intento: dirección positiva   */
    t_inicio = HAL_GetTick();
    motor1_set_velocidad(&htim5, vel_r1);
    estado   = HOMING_R1_BUSCAR;
}

bool         Homing_EstaCompleto(void) { return estado == HOMING_COMPLETO; }
HomingEstado Homing_GetEstado(void)    { return estado; }

bool Homing_EstaActivo(void) {
    return estado != HOMING_IDLE &&
           estado != HOMING_COMPLETO &&
           estado != HOMING_ERROR;
}

/* ── Tick principal ──────────────────────────────────────────────────────── */
void Homing_Tick(void) {
    uint32_t ahora = HAL_GetTick();

    switch (estado) {

    /* ── R1: mover hasta encontrar PA8 ──────────────────────────────────── */
    case HOMING_R1_BUSCAR:
        if (FinalDeCarrera_getFlag(&limiteInclinacion)) {
            FinalDeCarrera_resetFlag(&limiteInclinacion);
            motor1_set_velocidad(&htim5, 0);
            EncoderRobot_reset(&encoderInclinacion);   /* R1 = 0 en home     */
            motor1_set_velocidad(&htim5, -vel_r1);    /* separarse del tope */
            t_inicio = ahora;
            estado   = HOMING_R1_BACKOFF;
        } else if (ahora - t_inicio > TIMEOUT_R1_MS) {
            if (vel_r1 == VEL_R1_HOME) {
                /* primer intento fallido — probar dirección contraria */
                vel_r1   = -VEL_R1_HOME;
                t_inicio = ahora;
                motor1_set_velocidad(&htim5, vel_r1);
            } else {
                /* ambas direcciones fallaron */
                motor1_set_velocidad(&htim5, 0);
                estado = HOMING_ERROR;
            }
        }
        break;

    /* ── R1: alejarse del tope hasta BACKOFF_GRADOS ─────────────────────── */
    case HOMING_R1_BACKOFF:
        if (fabsf(angulo_abs_r1()) >= BACKOFF_GRADOS) {
            motor1_set_velocidad(&htim5, 0);
            /* El encoder queda referenciado: R1=0 en PA8, R1≈±5° ahora    */
            /* Iniciar búsqueda de home de traslación                       */
            dir_tras = 0;
            stepper_iniciar_movimiento(mm_a_pasos(600.0f), dir_tras, VEL_STEPPER_HOMING);
            t_inicio = ahora;
            estado   = HOMING_TRAS_BUSCAR;
        } else if (ahora - t_inicio > TIMEOUT_R1_MS) {
            motor1_set_velocidad(&htim5, 0);
            estado = HOMING_ERROR;
        }
        break;

    /* ── Traslación: mover hasta encontrar PA9 (auto-dirección) ─────────── */
    case HOMING_TRAS_BUSCAR:
        if (FinalDeCarrera_getFlag(&limiteTraslacion_A)) {
            /* PA9 encontrado — ésta era la dirección correcta */
            FinalDeCarrera_resetFlag(&limiteTraslacion_A);
            pap_estado.pasos_restantes = 0;
            EncoderRobot_reset(&encoderTraslacion);    /* base = 0 en PA9   */
            stepper_reset_posicion();
            /* separarse del tope en dirección contraria */
            stepper_iniciar_movimiento(
                mm_a_pasos(BACKOFF_MM), dir_tras ? 0u : 1u, VEL_STEPPER_HOMING);
            t_inicio = ahora;
            estado   = HOMING_TRAS_BACKOFF;

        } else if (FinalDeCarrera_getFlag(&limiteTraslacion_B)) {
            /* PA10 encontrado — íbamos en sentido incorrecto, invertir     */
            FinalDeCarrera_resetFlag(&limiteTraslacion_B);
            pap_estado.pasos_restantes = 0;
            dir_tras = dir_tras ? 0u : 1u;            /* invertir dirección */
            stepper_iniciar_movimiento(mm_a_pasos(600.0f), dir_tras, VEL_STEPPER_HOMING);
            t_inicio = ahora;                          /* reiniciar timeout  */

        } else if (ahora - t_inicio > TIMEOUT_TRAS_MS) {
            pap_estado.pasos_restantes = 0;
            estado = HOMING_ERROR;
        }
        break;

    /* ── Traslación: esperar separación del tope ─────────────────────────── */
    case HOMING_TRAS_BACKOFF:
        if (!stepper_en_movimiento()) {
            /* Después del backoff, referenciar posición final               */
            EncoderRobot_reset(&encoderTraslacion);
            stepper_reset_posicion();
            estado = HOMING_COMPLETO;
        } else if (ahora - t_inicio > TIMEOUT_TRAS_MS) {
            pap_estado.pasos_restantes = 0;
            estado = HOMING_ERROR;
        }
        break;

    case HOMING_IDLE:
    case HOMING_COMPLETO:
    case HOMING_ERROR:
        break;
    }
}
