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

void reset_motores(TIM_HandleTypeDef *htim);

#endif /* INC_MOTORES_H_ */
