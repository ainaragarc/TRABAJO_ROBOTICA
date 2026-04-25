#include "EncoderRobot.h"

void EncoderRobot_init(EncoderRobot* e, TIM_HandleTypeDef* htim, uint32_t ppr, float diametro) {
    e->htim = htim;
    e->ppr = ppr;
    e->diametroRueda = diametro;
    e->vueltas = 0;
}

void EncoderRobot_inicializar(EncoderRobot* e) {
    HAL_TIM_Encoder_Start(e->htim, TIM_CHANNEL_ALL);
}

void EncoderRobot_registrarVueltaZ(EncoderRobot* e) {
    if (e->htim->Instance->CR1 & TIM_CR1_DIR) {
        e->vueltas--;  // giro hacia atrás
    } else {
        e->vueltas++;  // giro hacia adelante
    }
    __HAL_TIM_SET_COUNTER(e->htim, 0);
}

void EncoderRobot_reset(EncoderRobot* e) {
    __disable_irq();
    __HAL_TIM_SET_COUNTER(e->htim, 0);
    e->vueltas = 0;
    __enable_irq();
}

int32_t EncoderRobot_getPulsosTotales(EncoderRobot* e) {
    __disable_irq();
    int32_t conteoHardware = (int32_t)__HAL_TIM_GET_COUNTER(e->htim);
    int32_t vueltas = e->vueltas;
    __enable_irq();
    return conteoHardware + (vueltas * (int32_t)e->ppr);
}

float EncoderRobot_getAnguloGrados(EncoderRobot* e) {
    return (float)(EncoderRobot_getPulsosTotales(e) % e->ppr) * (360.0f / e->ppr);
}

float EncoderRobot_getDistanciaMM(EncoderRobot* e) {
    return ((float)EncoderRobot_getPulsosTotales(e) / e->ppr) * (3.14159f * e->diametroRueda);
}
