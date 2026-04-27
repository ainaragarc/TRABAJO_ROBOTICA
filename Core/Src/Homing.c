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
#define SETTLE_MS           150u    /* A3: espera tras parar antes de reset  */

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
static int16_t      vel_r1    = VEL_R1_HOME;
static uint8_t      dir_tras  = 0;

/* ── API pública ─────────────────────────────────────────────────────────── */
void Homing_Iniciar(void) {
    /* A4: si R1 ya está pisando PA8 al arrancar, no hay flanco EXTI → ir
       directamente al backoff sin esperar timeout */
    if (FinalDeCarrera_estaPresionado(&limiteInclinacion)) {
        EncoderRobot_reset(&encoderInclinacion);
        vel_r1   =  VEL_R1_HOME;          /* guardar sentido que encontró home  */
        motor1_set_velocidad(&htim5, -vel_r1);  /* backoff inmediato            */
        t_inicio = HAL_GetTick();
        estado   = HOMING_R1_BACKOFF;
        return;
    }
    vel_r1   = VEL_R1_HOME;
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
    case HOMING_R1_BUSCAR: {
        /* A3: sub-espera después de parar, antes de resetear el encoder.
           Si el servo sigue girando por inercia cuando reseteamos, perdemos
           la posición real de home. */
        static uint32_t t_parada = 0;

        if (t_parada != 0) {
            if (ahora - t_parada >= SETTLE_MS) {
                EncoderRobot_reset(&encoderInclinacion);   /* R1 = 0 en home  */
                motor1_set_velocidad(&htim5, -vel_r1);    /* separarse        */
                t_inicio = ahora;
                t_parada = 0;
                estado   = HOMING_R1_BACKOFF;
            }
            break;
        }

        if (FinalDeCarrera_getFlag(&limiteInclinacion)) {
            FinalDeCarrera_resetFlag(&limiteInclinacion);
            motor1_set_velocidad(&htim5, 0);
            t_parada = ahora;   /* esperar SETTLE_MS antes de resetear */
        } else if (ahora - t_inicio > TIMEOUT_R1_MS) {
            if (vel_r1 == VEL_R1_HOME) {
                vel_r1   = -VEL_R1_HOME;
                t_inicio = ahora;
                motor1_set_velocidad(&htim5, vel_r1);
            } else {
                motor1_set_velocidad(&htim5, 0);
                estado = HOMING_ERROR;
            }
        }
        break;
    }

    /* ── R1: alejarse del tope hasta BACKOFF_GRADOS ─────────────────────── */
    case HOMING_R1_BACKOFF:
        /* C5: usar getAnguloGradosAbs en lugar del helper local */
        if (fabsf(EncoderRobot_getAnguloGradosAbs(&encoderInclinacion)) >= BACKOFF_GRADOS) {
            motor1_set_velocidad(&htim5, 0);
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
        /* A4: usar estaPresionado además de getFlag — si PA9 ya estaba activo
           al entrar al estado (sin flanco EXTI), getFlag=false pero el switch
           sí está pulsado. Combinar ambas condiciones lo cubre todo. */
        if (FinalDeCarrera_getFlag(&limiteTraslacion_A) ||
            FinalDeCarrera_estaPresionado(&limiteTraslacion_A)) {
            FinalDeCarrera_resetFlag(&limiteTraslacion_A);
            pap_estado.pasos_restantes = 0;
            EncoderRobot_reset(&encoderTraslacion);
            stepper_reset_posicion();
            stepper_iniciar_movimiento(
                mm_a_pasos(BACKOFF_MM), dir_tras ? 0u : 1u, VEL_STEPPER_HOMING);
            t_inicio = ahora;
            estado   = HOMING_TRAS_BACKOFF;

        } else if (FinalDeCarrera_getFlag(&limiteTraslacion_B) ||
                   FinalDeCarrera_estaPresionado(&limiteTraslacion_B)) {
            FinalDeCarrera_resetFlag(&limiteTraslacion_B);
            pap_estado.pasos_restantes = 0;
            dir_tras = dir_tras ? 0u : 1u;
            stepper_iniciar_movimiento(mm_a_pasos(600.0f), dir_tras, VEL_STEPPER_HOMING);
            t_inicio = ahora;

        } else if (ahora - t_inicio > TIMEOUT_TRAS_MS) {
            pap_estado.pasos_restantes = 0;
            estado = HOMING_ERROR;
        }
        break;

    /* ── Traslación: esperar separación del tope ─────────────────────────── */
    case HOMING_TRAS_BACKOFF:
        if (!stepper_en_movimiento()) {
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
