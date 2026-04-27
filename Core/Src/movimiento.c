#include "movimiento.h"

extern TIM_HandleTypeDef htim1;   /* revólver — TIM1_CH3 / PE13 */

/* ── Tipos ───────────────────────────────────────────────────────────────── */
typedef enum {
    MOV_IDLE = 0,
    MOV_VIAJE,          /* vuelo libre hasta la posición de aproximación     */
    MOV_APROXIMAR,      /* acercarse al lienzo (herramienta en contacto)     */
    MOV_DIBUJAR,        /* trazar sobre el lienzo punto a punto              */
    MOV_RETIRAR,        /* alejar la herramienta del lienzo                  */
    MOV_CAMBIO_COLOR,   /* girar el revólver y esperar a que llegue          */
    MOV_REPOSO,         /* volver a posición segura                          */
} EstadoMov;

/* ── Estado interno ──────────────────────────────────────────────────────── */
static EstadoMov estado       = MOV_IDLE;
static uint8_t   idx_punto    = 0;
static Color     color_actual = COLOR1;
static Color     color_inicio = COLOR1;
static bool      activo       = false;
static puntos    dibujo       = {0};
static uint32_t  t_color      = 0;

/* Posición de reposo: frente al lienzo, herramienta retirada */
static const c3d REPOSO = { .x = SEPx, .y = SEPy + 50, .z = SEPz };

/* ── Núcleo del sistema: un paso de cinemática → motores ────────────────── */
static bool ejecutar(c3d objetivo, uint8_t flag) {
    motoresg siguiente;
    bool llegue = trayectoria_cutre(&siguiente, objetivo, &flag);
    control_loop_motores(siguiente);
    return llegue;
}

/* ── Helpers ─────────────────────────────────────────────────────────────── */
static Color color_siguiente(Color c) {
    switch (c) {
    case COLOR1: return COLOR2;
    case COLOR2: return COLOR3;
    default:     return COLOR1;
    }
}

/* ── Utilidades públicas ─────────────────────────────────────────────────── */
void matriz_aleatoria(void) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < NUMERO_PUNTOS; j++) {
            dibujo.p[i][j].y = rand() % 200;
            dibujo.p[i][j].z = rand() % 200;
        }
}

bool cambio_color_revolver(Color c, TIM_HandleTypeDef *htim) {
    switch (c) {
    case COLOR1: set_servo_revolver(htim,  500u); break;
    case COLOR2: set_servo_revolver(htim, 1500u); break;
    case COLOR3: set_servo_revolver(htim, 2500u); break;
    }
    return true;
}

/* ── API de misión ───────────────────────────────────────────────────────── */
void movimiento_cargar_mision(void) {
    matriz_aleatoria();
    idx_punto    = 0;
    color_actual = COLOR1;
    color_inicio = COLOR1;
    activo       = false;
    estado       = MOV_IDLE;
    cambio_color_revolver(COLOR1, &htim1);  /* revólver al color inicial */
}

void movimiento_iniciar(void) {
    if (estado != MOV_IDLE) return;
    velocidad_reset();
    activo = true;
    estado = MOV_VIAJE;
}

void movimiento_parar(void) {
    activo = false;
    velocidad_reset();
    estado = MOV_REPOSO;
}

bool movimiento_activo(void) { return activo; }

/* ── Tick principal — llamar en cada iteración del loop ─────────────────── */
void movimiento_tick(void) {
    if (estado == MOV_IDLE) return;

    switch (estado) {

    /* 1. Volar hasta la posición de aproximación del punto actual */
    case MOV_VIAJE:
        if (ejecutar(plano_retroceso(dibujo.p[color_actual][idx_punto]), 0))
            estado = MOV_APROXIMAR;
        break;

    /* 2. Acercarse al lienzo (herramienta entra en contacto) */
    case MOV_APROXIMAR:
        if (ejecutar(plano_dibujo(dibujo.p[color_actual][idx_punto]), 1))
            estado = MOV_DIBUJAR;
        break;

    /* 3. Trazar sobre el lienzo avanzando punto a punto sin levantar */
    case MOV_DIBUJAR:
        if (ejecutar(plano_dibujo(dibujo.p[color_actual][idx_punto]), 2)) {
            if (idx_punto < NUMERO_PUNTOS - 1) {
                idx_punto++;          /* siguiente punto, la herramienta no se levanta */
            } else {
                estado = MOV_RETIRAR; /* último punto alcanzado */
            }
        }
        break;

    /* 4. Retirar la herramienta del lienzo */
    case MOV_RETIRAR:
        if (ejecutar(plano_retroceso(dibujo.p[color_actual][idx_punto]), 3)) {
            idx_punto    = 0;
            color_actual = color_siguiente(color_actual);
            cambio_color_revolver(color_actual, &htim1);
            t_color = HAL_GetTick();
            estado  = MOV_CAMBIO_COLOR;
        }
        break;

    /* 5. Esperar a que el revólver alcance la posición */
    case MOV_CAMBIO_COLOR:
        if (HAL_GetTick() - t_color >= 1000u) {
            if (color_actual == color_inicio) {
                /* completados todos los colores */
                activo = false;
                estado = MOV_REPOSO;
            } else {
                velocidad_reset();
                estado = MOV_VIAJE;
            }
        }
        break;

    /* 6. Volver a posición segura de reposo */
    case MOV_REPOSO:
        if (ejecutar(REPOSO, 0))
            estado = MOV_IDLE;
        break;

    default:
        estado = MOV_IDLE;
        break;
    }
}
