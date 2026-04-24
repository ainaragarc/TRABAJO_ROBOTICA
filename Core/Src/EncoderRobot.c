#include "EncoderRobot.h"

EncoderRobot::EncoderRobot(TIM_HandleTypeDef* htim, uint32_t ppr, float diametro) {
    _htim = htim;
    _ppr = ppr;
    _diametroRueda = diametro;
    _vueltas = 0;
}

void EncoderRobot::inicializar() {
    HAL_TIM_Encoder_Start(_htim, TIM_CHANNEL_ALL);
}

void EncoderRobot::registrarVueltaZ() {
    _vueltas++;
    __HAL_TIM_SET_COUNTER(_htim, 0);
}

void EncoderRobot::reset() {
    __disable_irq();
    __HAL_TIM_SET_COUNTER(_htim, 0);
    _vueltas = 0;
    __enable_irq();
}

int32_t EncoderRobot::getPulsosTotales() {
    // Lectura atómica: evita race condition entre el contador hardware y _vueltas
    // que se incrementa en la ISR del canal Z.
    __disable_irq();
    int16_t conteoHardware = (int16_t)__HAL_TIM_GET_COUNTER(_htim);
    int32_t vueltas = _vueltas;
    __enable_irq();
    return conteoHardware + (vueltas * (int32_t)_ppr);
}

float EncoderRobot::getAnguloGrados() {
    return (float)(getPulsosTotales() % _ppr) * (360.0f / _ppr);
}

float EncoderRobot::getDistanciaMM() {
    return ((float)getPulsosTotales() / _ppr) * (3.14159f * _diametroRueda);
}