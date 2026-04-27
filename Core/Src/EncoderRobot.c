#include "EncoderRobot.h"

void EncoderRobot_init(EncoderRobot* e, TIM_HandleTypeDef* htim, uint32_t ppr, float diametro) {
    e->htim         = htim;
    e->ppr          = ppr;
    e->diametroRueda = (diametro > 0.0f) ? diametro : 1.0f;  /* B8: guard diámetro ≤ 0 */
    e->vueltas      = 0;
}

void EncoderRobot_init_lineal(EncoderRobot* e, TIM_HandleTypeDef* htim,
                               uint32_t ppr, float mm_por_vuelta) {
    /* C1: API explícita para husillo lineal.
       getDistanciaMM = (pulsos/ppr) * π * d  →  d = mm_por_vuelta / π
       Así el llamador razona en mm/vuelta sin conocer el truco interno. */
    EncoderRobot_init(e, htim, ppr, mm_por_vuelta / 3.14159265f);
}

void EncoderRobot_inicializar(EncoderRobot* e) {
    HAL_TIM_Encoder_Start(e->htim, TIM_CHANNEL_ALL);
}

void EncoderRobot_registrarVueltaZ(EncoderRobot* e) {
    /* Usar el signo del contador en el momento de Z, no CR1_DIR, que refleja
       el último flanco AB y puede ser incorrecto si hay vibración cerca de Z. */
    int16_t cnt = (int16_t)__HAL_TIM_GET_COUNTER(e->htim);
    if      (cnt > 0) e->vueltas++;
    else if (cnt < 0) e->vueltas--;
    __HAL_TIM_SET_COUNTER(e->htim, 0);
}

void EncoderRobot_reset(EncoderRobot* e) {
    __disable_irq();
    __HAL_TIM_SET_COUNTER(e->htim, 0);
    e->vueltas = 0;
    __enable_irq();
}

int32_t EncoderRobot_getPulsosTotales(EncoderRobot* e) {
    // Lectura atómica: evita race condition entre el contador hardware y vueltas
    // que se incrementa en la ISR del canal Z.
    __disable_irq();
    int16_t conteoHardware = (int16_t)__HAL_TIM_GET_COUNTER(e->htim);
    int32_t vueltas = e->vueltas;
    __enable_irq();
    return conteoHardware + (vueltas * (int32_t)e->ppr);
}

float EncoderRobot_getAnguloGrados(EncoderRobot* e) {
    return (float)(EncoderRobot_getPulsosTotales(e) % e->ppr) * (360.0f / e->ppr);
}

float EncoderRobot_getAnguloGradosAbs(EncoderRobot* e) {
    /* Sin módulo: devuelve desplazamiento angular acumulado desde el último reset.
       Necesario para control de R1 — el módulo pierde el ángulo absoluto. */
    return (float)EncoderRobot_getPulsosTotales(e) * (360.0f / (float)e->ppr);
}

float EncoderRobot_getDistanciaMM(EncoderRobot* e) {
    return ((float)EncoderRobot_getPulsosTotales(e) / e->ppr) * (3.14159f * e->diametroRueda);
}
