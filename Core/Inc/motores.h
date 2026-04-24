#ifndef INC_MOTORES_H_
#define INC_MOTORES_H_

#include <stdint.h>
#include <stdbool.h>
#include "EncoderRobot.h"
#include "stm32f4xx_hal.h"

///////
// MOTOR 1: SERVO ROTACIONAL

typedef struct {
    bool en_movimiento;
    float angulo_objetivo;
    float angulo_inicial;
    int8_t velocidad;
} EstadoMotorRotacional;

extern EstadoMotorRotacional motor1_estado;


void motor1_set_velocidad(TIM_HandleTypeDef *htim, int8_t velocidad);
void motor1_mover_grados_estimados(TIM_HandleTypeDef *htim, float grados, int8_t velocidad_test);


///////
// MOTORES POSICIONALES:

void set_servo_2(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_3(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_revolver(TIM_HandleTypeDef *htim, uint16_t us);


uint16_t get_servo_2(void);
uint16_t get_servo_3(void);
uint16_t get_servo_revolver(void);



//////////
// MOTOR PASO A PASO:

typedef struct {
    uint32_t pasos_restantes;
    uint32_t ultimo_tick;
    uint16_t intervalo_ms;
    bool nivel_pin;
} EstadoStepper;

extern EstadoStepper pap_estado;

void stepper_iniciar_movimiento(uint32_t pasos, uint8_t horario, uint16_t velocidad_ms);
void stepper_control_tick(void);

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
