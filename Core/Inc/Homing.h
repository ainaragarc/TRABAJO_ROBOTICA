#ifndef HOMING_H
#define HOMING_H

#include "main.h"
#include <stdbool.h>

typedef enum {
    HOMING_IDLE,
    HOMING_TRASLACION,
    HOMING_RETROCESO_T,
    HOMING_INCLINACION,
    HOMING_RETROCESO_I,
    HOMING_COMPLETO,
    HOMING_ERROR
} HomingEstado;

void         Homing_Iniciar(void);
void         Homing_Tick(void);
bool         Homing_EstaCompleto(void);
bool         Homing_EstaActivo(void);
HomingEstado Homing_GetEstado(void);

#endif
