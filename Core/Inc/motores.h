#ifndef INC_MOTORES_H_
#define INC_MOTORES_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

void set_servo_1(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_2(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_revolver(TIM_HandleTypeDef *htim, uint16_t us);

uint16_t get_servo_1(void);
uint16_t get_servo_2(void);
uint16_t get_servo_revolver(void);



//CONVERSION MOTORES EN GRADOS EN ENTEROS Y VICVERSA

float convertir_grados (uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max);
uint16_t convertir_int (float x, float in_min, float in_max, uint16_t out_min, uint16_t out_max);

float grados_pos(uint16_t cord);
uint16_t entero_pos(float angulo);

float grados_revol(uint16_t cord);
uint16_t entero_revol(float angulo);

//paso a paso (VAN CON SENSORICA)
float grados_paso (uint16_t angulo);
uint16_t entero_paso (float angulo);

//servos controlados por rotacion (VAN CON SENSORICA)
float grados_rot (uint16_t cord);
uint16_t entero_rot (float angulo);

void reset_motores(TIM_HandleTypeDef *htim);

#endif /* INC_MOTORES_H_ */
