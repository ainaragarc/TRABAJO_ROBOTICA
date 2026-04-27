#ifndef INC_MOTORES_H_
#define INC_MOTORES_H_

#include <stdint.h>
#include <stdbool.h>
#include "EncoderRobot.h"
#include "stm32f4xx_hal.h"

// ── Constantes stepper ────────────────────────────────────────────────────────
#define PASOS_POR_VUELTA    200u
#define MM_POR_VUELTA       8.0f
#define PASOS_POR_MM        25u         // 200/8
#define MM_POR_PASO         0.04f       // 8/200

#define DT_CONTROL          0.01f

// ── Structs motores ───────────────────────────────────────────────────────────
typedef struct {
    int16_t base;   // pasos stepper
    int16_t r1;     // servo continuo inclinación
    int16_t r2;     // servo posicional codo
    int16_t r3;     // servo posicional muñeca
} motores;

// CAMBIO: base es mm (no grados), r1 es grados de inclinación
typedef struct {
    float base;     // mm de traslación desde home
    float r1;       // grados inclinación primer eslabón
    float r2;       // grados codo
    float r3;       // grados muñeca
} motoresg;

// ── PID ───────────────────────────────────────────────────────────────────────
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float vmax;
    float zonamuerta;
    float dt;   /* periodo real medido (s); 0 = usar DT_CONTROL */
} PID;

float pid_funcion(PID *pid, float error);

// ── Motor 1: servo rotacional (inclinación, TIM5_CH2) ────────────────────────
typedef struct {
    bool    en_movimiento;
    float   angulo_objetivo;
    float   angulo_inicial;
    int16_t velocidad;
} EstadoMotorRotacional;

extern EstadoMotorRotacional motor1_estado;

void motor1_set_velocidad(TIM_HandleTypeDef *htim, int16_t velocidad);
void motor1_parar(void);
void motor1_iniciar_giro(EncoderRobot *encoder, float grados, int16_t velocidad);
void motor1_control_tick(EncoderRobot *encoder);
void motor1_pid_tick(EncoderRobot *encoder, float objetivo_grados);
void motor1_mover_grados_estimados(TIM_HandleTypeDef *htim, float grados, int16_t velocidad_test);

// ── Servos posicionales (TIM5_CH3 codo, TIM5_CH4 muñeca) ────────────────────
void set_servo_2(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_3(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_revolver(TIM_HandleTypeDef *htim, uint16_t us);
void set_servo_2_grados(TIM_HandleTypeDef *htim, float grados);
void set_servo_3_grados(TIM_HandleTypeDef *htim, float grados);

uint16_t get_servo_2(void);
uint16_t get_servo_3(void);
uint16_t get_servo_revolver(void);
void reset_motores(TIM_HandleTypeDef *htim);

// ── Stepper + A4988 ───────────────────────────────────────────────────────────
typedef struct {
    uint32_t pasos_restantes;
    uint32_t ultimo_tick;
    uint16_t intervalo_ms;
    bool     nivel_pin;
    int32_t  posicion_pasos;  // posición absoluta en pasos desde home
} EstadoStepper;

extern EstadoStepper pap_estado;

void     stepper_iniciar_movimiento(uint32_t pasos, uint8_t horario, uint16_t velocidad_ms);
void     stepper_mover_mm(float mm, uint16_t velocidad_ms);
void     stepper_mover_a_mm(float mm_objetivo, uint16_t velocidad_ms);
void     stepper_control_tick(void);
bool     stepper_en_movimiento(void);
float    stepper_get_posicion_mm(void);
void     stepper_reset_posicion(void);
uint32_t mm_a_pasos(float mm);
float    pasos_a_mm(uint32_t pasos);

// ── Lectura y escritura de estado ─────────────────────────────────────────────
// CAMBIO: sin parámetros — usa globales encoderInclinacion/encoderTraslacion
motoresg get_motoresg(void);
bool     set_motores(motoresg m);
void     control_loop_motores(motoresg objetivo);

void freno_R1(float objetivo);
bool r1_llega(float objetivo_r1);

// ── Conversión ────────────────────────────────────────────────────────────────
float    convertir_grados(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max);
uint16_t convertir_int(float x, float in_min, float in_max, uint16_t out_min, uint16_t out_max);

float    grados_pos(uint16_t cord);
uint16_t entero_pos(float angulo);
float    grados_revol(uint16_t cord);
uint16_t entero_revol(float angulo);
float    grados_paso(uint16_t angulo);
uint16_t entero_paso(float angulo);
float    grados_rot(uint16_t cord);
uint16_t entero_rot(float angulo);

#endif /* INC_MOTORES_H_ */
