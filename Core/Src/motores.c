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


//////////////////////////////////////////////////////
//CONVERSION MOTORES EN GRADOS EN ENTEROS Y VICVERSA

float convertir_grados (uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max)
{
    float res = (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;

    if (res < out_min) x = out_min;
    if (res > out_max) x = out_max;

    return res;
}

uint16_t convertir_int (float x, float in_min, float in_max, uint16_t out_min, uint16_t out_max)
{
    float res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (res < out_min) x = out_min;
    if (res > out_max) x = out_max;

    return (uint16_t)(res + 0.5f);
}

float grados_pos(uint16_t cord){ return convertir_grados(cord, 500, 2500, 0.0f, 360.0f); }

uint16_t entero_pos(float angulo){ return convertir_int(angulo, 0.0f, 360.0f, 500, 2500); }

float grados_revol(uint16_t cord){ return convertir_grados(cord, 950, 1950, 0.0f, 180.0f); }

uint16_t entero_revol(float angulo){ return convertir_int(angulo, 0.0f, 180.0f, 950, 1950);}

//paso a paso (VAN CON SENSORICA)
float grados_paso (uint16_t angulo){
		float vuelta = angulo * 80;
		//FALTA LA CONVERSION DE LO QUE SEA QUE DE EL SENSOR

		return vuelta;

}

uint16_t entero_paso (float angulo){
		//FALTA LA LOGICA DEL SENSOR
		return angulo;

}

//servos controlados por rotacion (VAN CON SENSORICA)
float grados_rot (uint16_t cord){
	//FALTA LA CONVERSION DE LO QUE SEA QUE DE EL SENSOR
	return cord;

}

uint16_t entero_rot (float angulo){
	//FALTA LA CONVERSION DE LO QUE SEA QUE DE EL SENSOR
	return angulo;

}
