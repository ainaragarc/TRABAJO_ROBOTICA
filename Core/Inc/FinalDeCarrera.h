#ifndef FINAL_CARRERA_H
#define FINAL_CARRERA_H

#include "main.h"
#include <stdbool.h>

#define DEBOUNCE_MS 20u

typedef struct {
    GPIO_TypeDef* puerto;
    uint16_t      pin;
    bool          invertido;
    volatile bool flag;
    uint32_t      ultimoDisparo;
} FinalDeCarrera;

void     FinalDeCarrera_init(FinalDeCarrera* f, GPIO_TypeDef* puerto, uint16_t pin, bool invertido);
bool     FinalDeCarrera_estaPresionado(FinalDeCarrera* f);
void     FinalDeCarrera_onInterrupcion(FinalDeCarrera* f);
bool     FinalDeCarrera_getFlag(FinalDeCarrera* f);
void     FinalDeCarrera_resetFlag(FinalDeCarrera* f);
uint16_t FinalDeCarrera_getPin(FinalDeCarrera* f);

#endif

