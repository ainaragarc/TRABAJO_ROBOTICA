#include "motores.h"

#include <math.h>


//static uint16_t ANGULO_1 = 0u;
static uint16_t ANGULO_2 = 0u; // Ángulo del motor del codo (servo posicional)
static uint16_t ANGULO_3 = 0u;  // Ángulo del motor de la muñeca (servo posicional)
static uint16_t ANGULO_REVOLVER = 0u; // Ángulo del motor del revolver (servo posicional)

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;
extern bool peligroObstaculo;

///////
// MOTOR 1: SERVO ROTACIONAL

EstadoMotorRotacional motor1_estado = {0};

// Funciones sofisticadas con estructura estado motor
void motor1_iniciar_giro(EncoderRobot *encoder, float grados, int8_t velocidad) {
    motor1_estado.angulo_inicial = EncoderRobot_getAnguloGrados(encoder);
    motor1_estado.angulo_objetivo = grados;
    motor1_estado.velocidad = (grados > 0) ? velocidad : -velocidad;
    motor1_estado.en_movimiento = true;

    motor1_set_velocidad(&htim5, motor1_estado.velocidad);
}

void motor1_control_tick(EncoderRobot *encoder) {
    if (!motor1_estado.en_movimiento) return;

    float desplazamiento = EncoderRobot_getAnguloGrados(encoder) - motor1_estado.angulo_inicial;

    if (fabsf(desplazamiento) >= fabsf(motor1_estado.angulo_objetivo) || peligroObstaculo) {
        motor1_set_velocidad(&htim5, 0);
        motor1_estado.en_movimiento = false;
    }
}

// Funciones simples
void motor1_set_velocidad(TIM_HandleTypeDef *htim, int8_t velocidad) {
    // Mapeo simple:
    // 0 -> 1500us (parado)
    // 100 -> 2000us (aprox max adelante)
    // -100 -> 1000us (aprox max atrás)
    uint16_t us = 1500u + (velocidad * 5);

    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;

    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, us);
}

// Movimiento el lazo abierto: (solo para probar un pocao
void motor1_mover_grados_estimados(TIM_HandleTypeDef *htim, float grados, int8_t velocidad_test) {
    if (grados == 0 || velocidad_test == 0) return;

    // CALIBRACIÓN: Ajusta este valor.
    // Es el tiempo en ms que tarda el motor en mover 1 grado a 'velocidad_test'
    // Ejemplo: Si 360º son 2000ms -> 1º = 5.55ms
    const float MS_POR_GRADO = 5.55f;

    uint32_t tiempo_movimiento = (uint32_t)(fabsf(grados) * MS_POR_GRADO);

    // Determinar dirección
    if (grados > 0) {
        motor1_set_velocidad(htim, velocidad_test);
    } else {
        motor1_set_velocidad(htim, -velocidad_test);
    }

    HAL_Delay(tiempo_movimiento);

    // Frenar
    motor1_set_velocidad(htim, 0);
}


///////
// MOTORES POSICIONALES:

void set_servo_2(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion motor 2
	ANGULO_2=us;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, us);
}

void set_servo_3(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion motor 2
	ANGULO_3=us;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, us);
}
void set_servo_revolver(TIM_HandleTypeDef *htim, uint16_t us)
{
	// Establece la posicion motor del revolver
	ANGULO_REVOLVER=us;
	__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, us);
}

uint16_t get_servo_2(void) { return ANGULO_2; }
uint16_t get_servo_3(void) { return ANGULO_3; }
uint16_t get_servo_revolver(void) { return ANGULO_REVOLVER; }

void reset_motores(TIM_HandleTypeDef *htim){
	//Motores a sus posiciones iniciales
	set_servo_2(htim, 1500u); // Motor del codo a 90º
	set_servo_3(htim, 500u);  // Motor de la muñeca (no sé pero un ángulo para que no se joda)
	set_servo_revolver(htim, 500u);

}


//////////
// MOTOR PASO A PASO:

EstadoStepper pap_estado = {0};

void stepper_iniciar_movimiento(uint32_t pasos, uint8_t horario, uint16_t velocidad_ms) {
    HAL_GPIO_WritePin(DIR_PAP_GPIO_Port, DIR_PAP_Pin, horario ? GPIO_PIN_SET : GPIO_PIN_RESET);
    pap_estado.pasos_restantes = pasos;
    pap_estado.intervalo_ms = velocidad_ms;
    pap_estado.ultimo_tick = HAL_GetTick();
    pap_estado.nivel_pin = false;
}

void stepper_control_tick(void) {
    if (pap_estado.pasos_restantes == 0 || peligroObstaculo) return;

    uint32_t ahora = HAL_GetTick();
    if (ahora - pap_estado.ultimo_tick >= pap_estado.intervalo_ms) {
        pap_estado.ultimo_tick = ahora;

        // Alternar el pin de STEP para crear el flanco
        if (!pap_estado.nivel_pin) {
            HAL_GPIO_WritePin(STEP_PAP_GPIO_Port, STEP_PAP_Pin, GPIO_PIN_SET);
            pap_estado.nivel_pin = true;
        } else {
            HAL_GPIO_WritePin(STEP_PAP_GPIO_Port, STEP_PAP_Pin, GPIO_PIN_RESET);
            pap_estado.nivel_pin = false;
            pap_estado.pasos_restantes--; // Se completa un paso tras el flanco de bajada
        }
    }
}




//////////////////////////////////////////////////////
//CONVERSION MOTORES EN GRADOS EN ENTEROS Y VICVERSA

float convertir_grados (uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max)
{
    float res = (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;

    if (res < out_min) res = out_min;
    if (res > out_max) res = out_max;

    return res;
}

uint16_t convertir_int (float x, float in_min, float in_max, uint16_t out_min, uint16_t out_max)
{
    float res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

    if (res < out_min) res = out_min;
    if (res > out_max) res = out_max;

    return (uint16_t)(res + 0.5f);
}

float grados_pos(uint16_t cord){ return  (180 -convertir_grados(cord, 500, 2500, 0.0f, 180.0f)); }

uint16_t entero_pos(float angulo){ return (3000 - convertir_int(angulo, 0.0f, 180.0f, 500, 2500)); }

float grados_revol(uint16_t cord){ return convertir_grados(cord, 950, 1950, 0.0f, 180.0f); }

uint16_t entero_revol(float angulo){ return convertir_int(angulo, 0.0f, 180.0f, 950, 1950);}

//paso a paso (VAN CON SENSORICA)
float grados_paso (uint16_t angulo){
		//float vuelta = angulo * 80; //esto ya no lo tengo yo tan claro, REVISAR
		//FALTA LA CONVERSION DE LO QUE SEA QUE DE EL SENSOR

		return angulo;

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
