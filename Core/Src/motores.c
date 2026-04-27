#include "motores.h"
#include <math.h>
#include <stdlib.h>

/* ── Delay para el pulso STEP (~2µs a 100 MHz).
   A4988 necesita mínimo 1µs HIGH — este bucle es más que suficiente.
   Se usa volatile int para que el compilador no lo optimice. */
static void stepper_pulse_delay(void) {
    volatile int i = 200;
    while (i--) {}
}

void stepper_init(void) { /* reservado para DWT si se habilita en el futuro */ }

static uint16_t ANGULO_2        = 0u;
static uint16_t ANGULO_3        = 0u;
static uint16_t ANGULO_REVOLVER = 0u;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim1;   /* revólver — C2/A6 */
extern volatile bool peligroObstaculo;
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

static volatile bool  freno_R1_activo   = false;
static volatile float freno_R1_objetivo = 0.0f;

// ── Motor 1: servo rotacional inclinación (TIM5_CH2) ─────────────────────────

void motor1_set_velocidad(TIM_HandleTypeDef *htim, int16_t velocidad) {
    /* ── Protección térmica R1 ─────────────────────────────────────────── */
    static uint32_t t_carga  = 0;
    static uint32_t t_reposo = 0;
    static int      en_reposo = 0;
    uint32_t ahora = HAL_GetTick();
    if (t_carga == 0) t_carga = ahora;

    if (en_reposo) {
        if (ahora - t_reposo >= R1_DESCANSO_MS) { en_reposo = 0; t_carga = ahora; }
        else velocidad = 0;
    } else if (velocidad > R1_VEL_ALTA_UMBRAL || velocidad < -R1_VEL_ALTA_UMBRAL) {
        if (ahora - t_carga >= R1_MAX_CARGA_MS) { en_reposo = 1; t_reposo = ahora; velocidad = 0; }
    } else {
        t_carga = ahora;
    }

    /* ── Límites angulares: bloquear la dirección peligrosa ────────────── */
    float r1 = EncoderRobot_getAnguloGradosAbs(&encoderInclinacion);
    if (r1 <= R1_SW_LIMITE_MIN && velocidad < 0) velocidad = 0;
    if (r1 >= R1_SW_LIMITE_MAX && velocidad > 0) velocidad = 0;

    int32_t us = 1500 + (velocidad * 5);
    if (us < 500) us = 500;
    if (us > 2500) us = 2500;
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)us);
}

void motor1_parar(void) {
    motor1_set_velocidad(&htim5, 0);
}

/* M12: mide dt real para que el PID de R1 sea consistente con control_loop_motores */
void motor1_pid_tick(EncoderRobot *encoder, float objetivo_grados) {
    static uint32_t t_prev = 0;
    if (peligroObstaculo) { motor1_parar(); return; }
    uint32_t ahora = HAL_GetTick();
    float dt = (t_prev == 0) ? DT_CONTROL : (float)(ahora - t_prev) * 0.001f;
    if (dt < 0.001f || dt > 0.1f) dt = DT_CONTROL;
    t_prev       = ahora;
    pid_r1.dt    = dt;
    /* C5: usar ángulo absoluto — getAnguloGrados con módulo pierde el ángulo real */
    float actual = EncoderRobot_getAnguloGradosAbs(encoder);
    float error  = objetivo_grados - actual;
    float u      = pid_funcion(&pid_r1, error);
    motor1_set_velocidad(&htim5, (int16_t)u);
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
    set_servo_3(htim, 1500u);
    /* A6: revólver siempre en htim1 (TIM1_CH3/PE13), no en el htim de los servos */
    set_servo_revolver(&htim1, 1500u);
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

void stepper_iniciar_movimiento(uint32_t pasos, uint8_t horario, uint32_t velocidad_us) {
    int8_t nuevo_sentido = horario ? 1 : -1;
    uint32_t vel_obj = (velocidad_us < STEPPER_US_MIN) ? STEPPER_US_MIN : velocidad_us;

    /* Cambio de dirección: esperar setup time del A4988 (mín 200ns; usamos 2ms) */
    if (pap_estado.sentido != 0 && pap_estado.sentido != nuevo_sentido) {
        HAL_GPIO_WritePin(DIR_PAP_GPIO_Port, DIR_PAP_Pin,
                          horario ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_Delay(2);
        pap_estado.intervalo_us = STEPPER_US_ARRANQUE;  /* reiniciar rampa */
    } else {
        HAL_GPIO_WritePin(DIR_PAP_GPIO_Port, DIR_PAP_Pin,
                          horario ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if (pap_estado.pasos_restantes == 0)
            pap_estado.intervalo_us = STEPPER_US_ARRANQUE; /* arrancaba parado */
    }

    pap_estado.sentido            = nuevo_sentido;
    pap_estado.pasos_restantes    = pasos;
    pap_estado.intervalo_obj_us   = vel_obj;
    pap_estado.ultimo_step_cyc    = HAL_GetTick();   /* se compara en ms */
    pap_estado.pasos_desde_cambio = 0;
}

void stepper_mover_mm(float mm, uint32_t velocidad_us) {
    if (fabsf(mm) < 0.001f) return;
    stepper_iniciar_movimiento(mm_a_pasos(mm), (mm > 0) ? 1 : 0, velocidad_us);
}

void stepper_mover_a_mm(float mm_objetivo, uint32_t velocidad_us) {
    float delta = mm_objetivo - stepper_get_posicion_mm();
    stepper_mover_mm(delta, velocidad_us);
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
            /* M2: usar sentido guardado en el struct, no leer GPIO (race con cmd cambiado) */
            pap_estado.posicion_pasos += pap_estado.sentido;
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
        /* dentro de tolerancia: cancelar y sincronizar contador open-loop con encoder (C3) */
        pap_estado.pasos_restantes = 0;
        pap_estado.posicion_pasos  = (int32_t)(base_actual * PASOS_POR_MM);
    } else if (!stepper_en_movimiento() && fabsf(u_base) > 0.1f) {
        uint16_t intervalo = (uint16_t)(1000.0f / (fabsf(u_base) * PASOS_POR_MM));
        if (intervalo < 2)   intervalo = 2;    /* vel. máx ~20 mm/s   */
        if (intervalo > 100) intervalo = 100;  /* vel. mín ~0.4 mm/s  */
        float delta = e_base;
        if (delta >  5.0f) delta =  5.0f;     /* máx 5 mm por ciclo PID */
        if (delta < -5.0f) delta = -5.0f;
        stepper_mover_mm(delta, intervalo);
    }

    /* Inclinación: objetivo.r1 viene de la IK optimizada (lo más cerca de 90°
       que permita alcanzar el punto objetivo dentro del rango [80°,130°]) */
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
    /* C5: ángulo absoluto — el módulo de getAnguloGrados daría saltos en extremos */
    float actual = EncoderRobot_getAnguloGradosAbs(&encoderInclinacion);
    return fabsf(actual - objetivo_r1) < 2.0f;
}

void freno_R1(float objetivo)
{
    freno_R1_activo   = true;
    freno_R1_objetivo = objetivo;
    /* C5: ángulo absoluto para evitar saltos en extremos del rango */
    float actual = EncoderRobot_getAnguloGradosAbs(&encoderInclinacion);
    float error  = objetivo - actual;

    if (fabsf(error) < 3.0f) {
        motor1_set_velocidad(&htim5, 0);
        /* A5: NO resetear integral — mantiene compensación acumulada de gravedad */
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
    /* M13: NaN se propaga a UB en el cast a uint16_t — guard explícito */
    if (isnan(x) || isinf(x)) return out_min;
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
