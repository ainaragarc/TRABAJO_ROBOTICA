#ifndef INC_MOVIMIENTO_H_
#define INC_MOVIMIENTO_H_

#include "cinematica.h"
#include "motores.h"
#include <stdlib.h>

#define NUMERO_PUNTOS   9u

typedef enum { COLOR1 = 0, COLOR2, COLOR3 } Color;

typedef struct {
    c2d p[3][NUMERO_PUNTOS];
} puntos;

/* ── Utilidades (siguen disponibles para tests externos) ─────────────────── */
void matriz_aleatoria(void);
bool cambio_color_revolver(Color c, TIM_HandleTypeDef *htim);

/* ── API de misión ───────────────────────────────────────────────────────── */
/*
 *  Flujo de uso:
 *    movimiento_cargar_mision()  →  genera los puntos de dibujo
 *    movimiento_iniciar()        →  arranca la secuencia
 *    movimiento_tick()           →  llamar en cada iteración del loop
 *    movimiento_parar()          →  cancela y va a reposo
 *    movimiento_activo()         →  true mientras haya misión en curso
 */
void movimiento_cargar_mision(void);
void movimiento_iniciar(void);
void movimiento_parar(void);
void movimiento_tick(void);
bool movimiento_activo(void);

#endif /* INC_MOVIMIENTO_H_ */
