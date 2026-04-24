#ifndef HOMING_H
#define HOMING_H

#include "main.h"

typedef enum {
    HOMING_IDLE,
    HOMING_TRASLACION,    // moviendo hacia fin de carrera de traslación
    HOMING_RETROCESO_T,   // alejándose del switch, esperando 5 mm
    HOMING_INCLINACION,   // moviendo hacia fin de carrera de inclinación
    HOMING_RETROCESO_I,   // alejándose del switch, esperando 5 mm
    HOMING_COMPLETO,
    HOMING_ERROR
} HomingEstado;

void        Homing_Iniciar(void);
void        Homing_Tick(void);
bool        Homing_EstaCompleto(void);
bool        Homing_EstaActivo(void);
HomingEstado Homing_GetEstado(void);

#endif
