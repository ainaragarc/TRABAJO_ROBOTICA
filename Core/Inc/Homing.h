#ifndef HOMING_H
#define HOMING_H

#include "main.h"
#include <stdbool.h>

typedef enum {
    HOMING_IDLE = 0,
    HOMING_R1_BUSCAR,       /* buscando PA8 en inclinación (auto-dirección)  */
    HOMING_R1_BACKOFF,      /* separándose de PA8                            */
    HOMING_TRAS_BUSCAR,     /* buscando PA9 en traslación (auto-dirección)   */
    HOMING_TRAS_BACKOFF,    /* separándose de PA9                            */
    HOMING_COMPLETO,
    HOMING_ERROR,
} HomingEstado;

void         Homing_Iniciar(void);
void         Homing_Tick(void);
bool         Homing_EstaCompleto(void);
bool         Homing_EstaActivo(void);
HomingEstado Homing_GetEstado(void);

#endif /* HOMING_H */
