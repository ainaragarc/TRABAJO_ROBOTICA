#ifndef INC_MOVIMIENTO_H_
#define INC_MOVIMIENTO_H_

#include "cinematica.h"
#include "motores.h"
#include <stdlib.h>

#define NUMERO_PUNTOS 9u

typedef enum { COLOR1 = 0, COLOR2, COLOR3 } Color;

typedef struct {
    c2d p[3][NUMERO_PUNTOS];
} puntos;

bool cambio_color_revolver(Color c, TIM_HandleTypeDef *htim);
void matriz_aleatoria(void);

void dibujar(motores *movimiento , uint8_t flag_parada, uint8_t flag_ON, TIM_HandleTypeDef *htim_revolver);

#endif /* INC_MOVIMIENTO_H_ */
