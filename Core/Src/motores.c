#include "motores.h"
#include <math.h>

static uint16_t ANGULO_2        = 0u;
static uint16_t ANGULO_3        = 0u;
static uint16_t ANGULO_REVOLVER = 0u;

extern TIM_HandleTypeDef htim5;
// CAMBIO: eliminado extern htim1 — ya no se usa
extern bool peligroObstaculo;
extern EncoderRobot encoderInclinacion;
extern EncoderRobot encoderTraslacion;

static PID pid_base = {
    .Kp = 2.0f, .Ki = 0.5f, .Kd = 0.1f,
    .integral = 0, .prev_error = 0,
    .vmax = 50.0f, .zonamuerta = 0.5f
};

static PID pid_r1 = {
    .Kp = 1.5f, .Ki = 0.3f, .Kd = 0.05f,
    .integral = 0, .prev_error = 0,
    .vmax = 30.0f, .zonamuerta = 0.5f
};

static bool freno_R1_activo = false;
static float freno_R1_objetivo = 0.0f;

// ── Motor 1: servo rotacional inclinación (TIM5_CH2) ─────────────────────────

EstadoMotorRotacional motor1_estado = {0};

void motor1_set_velocidad(TIM_HandleTypeDef *htim, int16_t velocidad) {
    int32_t us = 1500 + (velocidad * 5);
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)us);
}

// CAMBIO: añadida motor1_parar que faltaba en este .c
void motor1_parar(void) {
    motor1_set_velocidad(&htim5, 0);
    motor1_estado.en_movimiento = false;
}

void motor1_iniciar_giro(EncoderRobot *encoder, float grados, int16_t velocidad) {
    motor1_estado.angulo_inicial  = EncoderRobot_getAnguloGrados(encoder);
    motor1_estado.angulo_objetivo = grados;
    motor1_estado.velocidad       = (grados > 0) ? velocidad : -velocidad;
    motor1_estado.en_movimiento   = true;
    motor1_set_velocidad(&htim5, motor1_estado.velocidad);
}

void motor1_control_tick(EncoderRobot *encoder) {
    if (!motor1_estado.en_movimiento) return;
    float desplazamiento = EncoderRobot_getAnguloGrados(encoder) - motor1_estado.angulo_inicial;
    if (fabsf(desplazamiento) >= fabsf(motor1_estado.angulo_objetivo) || peligroObstaculo) {
        motor1_parar();
    }
}

void motor1_pid_tick(EncoderRobot *encoder, float objetivo_grados) {
    if (peligroObstaculo) { motor1_parar(); return; }
    float actual = EncoderRobot_getAnguloGrados(encoder);
    float error  = objetivo_grados - actual;
    float u      = pid_funcion(&pid_r1, error);
    motor1_set_velocidad(&htim5, (int16_t)u);
}

void motor1_mover_grados_estimados(TIM_HandleTypeDef *htim, float grados, int16_t velocidad_test) {
    if (grados == 0 || velocidad_test == 0) return;
    const float MS_POR_GRADO = 8.35f;
    uint32_t tiempo = (uint32_t)(fabsf(grados) * MS_POR_GRADO);
    motor1_set_velocidad(htim, (grados > 0) ? velocidad_test : -velocidad_test);
    HAL_Delay(tiempo);
    motor1_set_velocidad(htim, 0);
}

// ── Servos posicionales (TIM5_CH3 codo, TIM5_CH4 muñeca) ─────────────────────

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

// CAMBIO: añadidas funciones en grados para uso directo desde control
void set_servo_2_grados(TIM_HandleTypeDef *htim, float grados) {
    set_servo_2(htim, entero_pos(grados));
}

void set_servo_3_grados(TIM_HandleTypeDef *htim, float grados) {
    set_servo_3(htim, entero_pos(grados));
}

uint16_t get_servo_2(void)        { return ANGULO_2; }
uint16_t get_servo_3(void)        { return ANGULO_3; }
uint16_t get_servo_revolver(void) { return ANGULO_REVOLVER; }

void reset_motores(TIM_HandleTypeDef *htim) {
    set_servo_2(htim, 1500u);
    set_servo_3(htim, 1500u);  /* neutral — 500u era extremo mecánico */
    set_servo_revolver(htim, 500u);
    motor1_set_velocidad(htim, 0);
}

// ── Stepper + A4988 ───────────────────────────────────────────────────────────

EstadoStepper pap_estado = {0};

uint32_t mm_a_pasos(float mm) {
    return (uint32_t)(fabsf(mm) * PASOS_POR_MM);
}

float pasos_a_mm(uint32_t pasos) {
    return (float)pasos * MM_POR_PASO;
}

void stepper_iniciar_movimiento(uint32_t pasos, uint8_t horario, uint16_t velocidad_ms) {
    HAL_GPIO_WritePin(DIR_PAP_GPIO_Port, DIR_PAP_Pin,
                      horario ? GPIO_PIN_SET : GPIO_PIN_RESET);
    pap_estado.pasos_restantes = pasos;
    pap_estado.intervalo_ms    = velocidad_ms;
    pap_estado.ultimo_tick     = HAL_GetTick();
    pap_estado.nivel_pin       = false;
}

// CAMBIO: añadida stepper_mover_mm — lanza movimiento en mm directamente
void stepper_mover_mm(float mm, uint16_t velocidad_ms) {
    if (fabsf(mm) < 0.001f) return;
    stepper_iniciar_movimiento(mm_a_pasos(mm), (mm > 0) ? 1 : 0, velocidad_ms);
}

// CAMBIO: añadida stepper_mover_a_mm — mueve a posición absoluta
void stepper_mover_a_mm(float mm_objetivo, uint16_t velocidad_ms) {
    float delta = mm_objetivo - stepper_get_posicion_mm();
    stepper_mover_mm(delta, velocidad_ms);
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
            // CAMBIO: actualizar posición absoluta al contar cada paso
            uint8_t horario = HAL_GPIO_ReadPin(DIR_PAP_GPIO_Port, DIR_PAP_Pin);
            if (horario) pap_estado.posicion_pasos++;
            else         pap_estado.posicion_pasos--;
        }
    }
}

bool stepper_en_movimiento(void) {
    return pap_estado.pasos_restantes > 0;
}

float stepper_get_posicion_mm(void) {
    return (float)pap_estado.posicion_pasos * MM_POR_PASO;
}

void stepper_reset_posicion(void) {
    pap_estado.posicion_pasos = 0;
}

// ── Lectura estado completo ───────────────────────────────────────────────────

// CAMBIO: get_motoresg sin parámetros — usa globales encoderInclinacion/encoderTraslacion
// CAMBIO: m.base ahora es getDistanciaMM (mm), no ángulo
// CAMBIO: m.r1 es ángulo de inclinación desde encoderTraslacion
motoresg get_motoresg(void) {
    motoresg m;
    m.base = EncoderRobot_getDistanciaMM(&encoderTraslacion);  // mm de traslación
    m.r1   = EncoderRobot_getAnguloGrados(&encoderInclinacion); // grados inclinación
    m.r2   = grados_pos(get_servo_2());              // grados codo
    m.r3   = grados_pos(get_servo_3());              // grados muñeca
    return m;
}

bool set_motores(motoresg m) {
    motor1_pid_tick(&encoderInclinacion, m.r1);
    set_servo_2(&htim5, entero_pos(m.r2));
    set_servo_3(&htim5, entero_pos(m.r3));
    stepper_mover_a_mm(m.base, 3);
    return false;
}

// ── PID ───────────────────────────────────────────────────────────────────────

float pid_funcion(PID *pid, float error) {
    if (fabsf(error) < pid->zonamuerta) return 0.0f;

    float dt = (pid->dt > 0.0f) ? pid->dt : DT_CONTROL;

    pid->integral += error * dt;
    if (pid->integral >  pid->vmax) pid->integral =  pid->vmax;
    if (pid->integral < -pid->vmax) pid->integral = -pid->vmax;

    float derivada = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float u = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivada;
    if (u >  pid->vmax) u =  pid->vmax;
    if (u < -pid->vmax) u = -pid->vmax;
    return u;
}

void control_loop_motores(motoresg objetivo) {
    if (peligroObstaculo) { motor1_parar(); return; }

    /* Medir dt real para que los PIDs trabajen con el tiempo correcto */
    static uint32_t t_prev = 0;
    uint32_t ahora = HAL_GetTick();
    float dt = (t_prev == 0) ? DT_CONTROL : (float)(ahora - t_prev) * 0.001f;
    if (dt < 0.001f || dt > 0.1f) dt = DT_CONTROL;
    t_prev = ahora;
    pid_base.dt = dt;
    pid_r1.dt   = dt;

    /* Traslación: PID sobre distancia en mm */
    float base_actual = EncoderRobot_getDistanciaMM(&encoderTraslacion);
    float e_base      = objetivo.base - base_actual;
    float u_base      = pid_funcion(&pid_base, e_base);

    if (fabsf(e_base) < 1.0f) {
        /* dentro de tolerancia: cancelar movimiento pendiente */
        pap_estado.pasos_restantes = 0;
    } else if (!stepper_en_movimiento() && fabsf(u_base) > 0.1f) {
        uint16_t intervalo = (uint16_t)(1000.0f / (fabsf(u_base) * PASOS_POR_MM));
        if (intervalo < 2)   intervalo = 2;    /* vel. máx ~20 mm/s   */
        if (intervalo > 100) intervalo = 100;  /* vel. mín ~0.4 mm/s  */
        float delta = e_base;
        if (delta >  5.0f) delta =  5.0f;     /* máx 5 mm por ciclo PID */
        if (delta < -5.0f) delta = -5.0f;
        stepper_mover_mm(delta, intervalo);
    }

    /* Inclinación: freno activo cuando llega al objetivo, PID mientras viaja */
    if (freno_R1_activo && fabsf(objetivo.r1 - freno_R1_objetivo) > 1.0f) freno_R1_activo = false;
    if (!freno_R1_activo && r1_llega(objetivo.r1)) {
        freno_R1_objetivo = objetivo.r1;
        freno_R1_activo   = true;
    }
    if (freno_R1_activo) { freno_R1(freno_R1_objetivo); }
    else                 { motor1_pid_tick(&encoderInclinacion, objetivo.r1); }

    /* Servos posicionales: directo a posición — no tocar, funcionan bien */
    set_servo_2_grados(&htim5, objetivo.r2);
    set_servo_3_grados(&htim5, objetivo.r3);
}

bool r1_llega(float objetivo_r1)
{
    float actual = EncoderRobot_getAnguloGrados(&encoderInclinacion);
    float tol = 2.0f; // tolerancia en grados

    return fabsf(actual - objetivo_r1) < tol;
}

void freno_R1(float objetivo)
{
    freno_R1_activo   = true;
    freno_R1_objetivo = objetivo;
    float actual = EncoderRobot_getAnguloGrados(&encoderInclinacion);
    float error  = objetivo - actual;

    if (fabsf(error) < 1.0f) {
        motor1_set_velocidad(&htim5, 0);
        pid_r1.integral = 0;
        return;
    }

    float u = pid_funcion(&pid_r1, error);
    if (u >  20.0f) u =  20.0f;
    if (u < -20.0f) u = -20.0f;
    motor1_set_velocidad(&htim5, (int16_t)u);
}


// ── Conversión ────────────────────────────────────────────────────────────────

float convertir_grados(uint16_t x, uint16_t in_min, uint16_t in_max,
                       float out_min, float out_max) {
    float res = (float)(x - in_min) * (out_max - out_min) /
                (float)(in_max - in_min) + out_min;
    if (res < out_min) res = out_min;
    if (res > out_max) res = out_max;
    return res;
}

uint16_t convertir_int(float x, float in_min, float in_max,
                       uint16_t out_min, uint16_t out_max) {
    float res = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (res < out_min) res = out_min;
    if (res > out_max) res = out_max;
    return (uint16_t)(res + 0.5f);
}

float    grados_pos(uint16_t cord)    { return (180.0f - convertir_grados(cord, 500, 2500, 0.0f, 180.0f)); }
uint16_t entero_pos(float angulo)     { return (3000u  - convertir_int(angulo, 0.0f, 180.0f, 500, 2500)); }
float    grados_revol(uint16_t cord)  { return convertir_grados(cord, 950, 1950, 0.0f, 180.0f); }
uint16_t entero_revol(float angulo)   { return convertir_int(angulo, 0.0f, 180.0f, 950, 1950); }
float    grados_paso(uint16_t angulo) { return (float)angulo; }
uint16_t entero_paso(float angulo)    { return (uint16_t)angulo; }
float    grados_rot(uint16_t cord)    { return (float)cord; }
uint16_t entero_rot(float angulo)     { return (uint16_t)angulo; }
