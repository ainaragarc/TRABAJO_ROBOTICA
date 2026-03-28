#include "motores.h"

static uint16_t ANGULO_1 = 0u;
static uint16_t ANGULO_2 = 0u;
static uint16_t ANGULO_REVOLVER = 0u;

void set_servo_1(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion motor 1
	ANGULO_1=us;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, us);
}

void set_servo_2(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion motor 2
	ANGULO_2=us;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, us);
}

void set_servo_revolver(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion motor del revolver
	ANGULO_REVOLVER=us;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, us);
}

uint16_t get_servo_1(void) { return ANGULO_1; }
uint16_t get_servo_2(void) { return ANGULO_2; }
uint16_t get_servo_revolver(void) { return ANGULO_REVOLVER; }

void reset_motores(TIM_HandleTypeDef *htim){

	set_servo_1(htim, 500u); //creo que 500 era el misnimo, revisare
	set_servo_2(htim, 500u);
	set_servo_revolver(htim, 500u);

}

