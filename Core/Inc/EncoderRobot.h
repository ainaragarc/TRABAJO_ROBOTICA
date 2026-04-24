#ifndef ENCODER_ROBOT_H
#define ENCODER_ROBOT_H

#include "main.h"

class EncoderRobot {
private:
    TIM_HandleTypeDef* _htim;
    uint32_t _ppr;
    float _diametroRueda;
    volatile int32_t _vueltas;

public:
    EncoderRobot(TIM_HandleTypeDef* htim, uint32_t ppr, float diametro);
    void inicializar();
    void registrarVueltaZ();
    void reset();
    int32_t getPulsosTotales();
    float getAnguloGrados();
    float getDistanciaMM();
};

#endif