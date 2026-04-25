#include "motores.h"
#include <math.h>

static uint16_t ANGULO_2 = 0u;
static uint16_t ANGULO_3 = 0u;
static uint16_t ANGULO_REVOLVER = 0u;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;
extern bool peligroObstaculo;
extern EncoderRobot encIzq;
extern EncoderRobot encDer;

static PID pid_base = {
    .Kp = 2.0f,
    .Ki = 0.5f,
    .Kd = 0.1f,
    .integral = 0,
    .prev_error = 0,
    .vmax = 50.0f,
    .zonamuerta = 0.5f
};

static PID pid_r1 = {
    .Kp = 1.5f,
    .Ki = 0.3f,
    .Kd = 0.05f,
    .integral = 0,
    .prev_error = 0,
    .vmax = 30.0f,
    .zonamuerta = 0.5f
};


///////
// MOTOR 1: SERVO ROTACIONAL

EstadoMotorRotacional motor1_estado = {0};

void motor1_iniciar_giro(EncoderRobot *encoder, float grados, int16_t velocidad) {
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

void motor1_set_velocidad(TIM_HandleTypeDef *htim, int16_t velocidad) {
    uint16_t us = 1500u + (velocidad * 5);
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, us);
}

void motor1_mover_grados_estimados(TIM_HandleTypeDef *htim, float grados, int16_t velocidad_test) {
    if (grados == 0 || velocidad_test == 0) return;

    // CALIBRACIÓN: tiempo en ms por grado a velocidad_test
    // Dev A usaba 8.35f, Dev B usaba 5.55f → usamos 8.35f (más calibrado)
    const float MS_POR_GRADO = 8.35f;

    uint32_t tiempo_movimiento = (uint32_t)(fabsf(grados) * MS_POR_GRADO);

    if (grados > 0) {
        motor1_set_velocidad(htim, velocidad_test);
    } else {
        motor1_set_velocidad(htim, -velocidad_test);
    }

    HAL_Delay(tiempo_movimiento);
    motor1_set_velocidad(htim, 0);
}


///////
// MOTORES POSICIONALES:

void set_servo_2(TIM_HandleTypeDef *htim, uint16_t us) {
    ANGULO_2 = us;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, us);
}

void set_servo_3(TIM_HandleTypeDef *htim, uint16_t us) {
    ANGULO_3 = us;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, us);
}

void set_servo_revolver(TIM_HandleTypeDef *htim, uint16_t us) {
    ANGULO_REVOLVER = us;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, us);
}

uint16_t get_servo_2(void) { return ANGULO_2; }
uint16_t get_servo_3(void) { return ANGULO_3; }
uint16_t get_servo_revolver(void) { return ANGULO_REVOLVER; }

void reset_motores(TIM_HandleTypeDef *htim) {
    set_servo_2(htim, 1500u);
    set_servo_3(htim, 500u);
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

        if (!pap_estado.nivel_pin) {
            HAL_GPIO_WritePin(STEP_PAP_GPIO_Port, STEP_PAP_Pin, GPIO_PIN_SET);
            pap_estado.nivel_pin = true;
        } else {
            HAL_GPIO_WritePin(STEP_PAP_GPIO_Port, STEP_PAP_Pin, GPIO_PIN_RESET);
            pap_estado.nivel_pin = false;
            pap_estado.pasos_restantes--;
        }
    }
}


//////////////////////////////////////////////////////
// CONVERSION MOTORES EN GRADOS EN ENTEROS Y VICEVERSA

float convertir_grados(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max) {
    float res = (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
    if (res < out_min) res = out_min;
    if (res > out_max) res = out_max;
    return res;
}

uint16_t convertir_int(float x, float in_min, float in_max, uint16_t out_min, uint16_t out_max) {
    float res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (res < out_min) res = out_min;
    if (res > out_max) res = out_max;
    return (uint16_t)(res + 0.5f);
}

float grados_pos(uint16_t cord)     { return (180 - convertir_grados(cord, 500, 2500, 0.0f, 180.0f)); }
uint16_t entero_pos(float angulo)   { return (3000 - convertir_int(angulo, 0.0f, 180.0f, 500, 2500)); }
float grados_revol(uint16_t cord)   { return convertir_grados(cord, 950, 1950, 0.0f, 180.0f); }
uint16_t entero_revol(float angulo) { return convertir_int(angulo, 0.0f, 180.0f, 950, 1950); }

// paso a paso (VAN CON SENSORICA)
float grados_paso(uint16_t angulo)  { return angulo; }
uint16_t entero_paso(float angulo)  { return angulo; }

// servos controlados por rotacion (VAN CON SENSORICA)
float grados_rot(uint16_t cord)     { return cord; }
uint16_t entero_rot(float angulo)   { return angulo; }


//////////////////////////////////////////////////////
// LECTURA Y ESCRITURA DE MOTORES

motoresg get_motoresg(void) {
    motoresg m;
    m.base = EncoderRobot_getAnguloGrados(&encIzq);
    m.r1   = EncoderRobot_getAnguloGrados(&encDer);
    m.r2   = grados_pos(get_servo_2());
    m.r3   = grados_pos(get_servo_3());
    return m;
}

bool set_motores(motoresg m) {
    motor1_set_velocidad(&htim5, (int16_t)m.r1);
    set_servo_2(&htim1, entero_pos(m.r2));
    set_servo_3(&htim1, entero_pos(m.r3));
    return false;
}


//////////////////////////////////////////////////////
// CONTROL PID

float pid_funcion(PID *pid, float error) {
    if (fabsf(error) < pid->zonamuerta)
        return 0.0f;

    pid->integral += error * DT_CONTROL;
    if (pid->integral >  pid->vmax) pid->integral =  pid->vmax;
    if (pid->integral < -pid->vmax) pid->integral = -pid->vmax;

    float derivada = (error - pid->prev_error) / DT_CONTROL;
    pid->prev_error = error;  // corregido: era pid_base.prev_error en el original

    float u = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivada;
    if (u >  pid->vmax) u =  pid->vmax;
    if (u < -pid->vmax) u = -pid->vmax;

    return u;
}

void control_loop_motores(motoresg objetivo) {
    float r1_actual   = EncoderRobot_getAnguloGrados(&encDer);
    float base_actual = EncoderRobot_getAnguloGrados(&encIzq);

    float e_r1   = objetivo.r1   - r1_actual;
    float e_base = objetivo.base - base_actual;

    float u_base = pid_funcion(&pid_base, e_base);
    float u_r1   = pid_funcion(&pid_r1,   e_r1);

    if (fabsf(e_base) < 0.5f) u_base = 0;
    if (fabsf(e_r1)   < 0.5f) u_r1   = 0;

    motoresg salida;
    salida.r1   = u_r1;
    salida.base = u_base;
    salida.r2   = objetivo.r2;
    salida.r3   = objetivo.r3;

    set_motores(salida);
}
