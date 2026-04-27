#ifndef ENCODER_ROBOT_H
#define ENCODER_ROBOT_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t ppr;
    float diametroRueda;
    volatile int32_t vueltas;
} EncoderRobot;

void EncoderRobot_init(EncoderRobot* e, TIM_HandleTypeDef* htim, uint32_t ppr, float diametro);
/* Para ejes lineales: pasa mm_por_vuelta directamente en vez del diámetro físico */
void EncoderRobot_init_lineal(EncoderRobot* e, TIM_HandleTypeDef* htim, uint32_t ppr, float mm_por_vuelta);
void EncoderRobot_inicializar(EncoderRobot* e);
void EncoderRobot_registrarVueltaZ(EncoderRobot* e);
void EncoderRobot_reset(EncoderRobot* e);
int32_t EncoderRobot_getPulsosTotales(EncoderRobot* e);
float   EncoderRobot_getAnguloGrados(EncoderRobot* e);    /* con módulo PPR — uso interno */
float   EncoderRobot_getAnguloGradosAbs(EncoderRobot* e); /* sin módulo — para control R1 */
float   EncoderRobot_getDistanciaMM(EncoderRobot* e);

#endif
