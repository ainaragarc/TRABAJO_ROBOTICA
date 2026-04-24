#include "FinalDeCarrera.h"
#include <stdbool.h>

void FinalDeCarrera_init(FinalDeCarrera* f, GPIO_TypeDef* puerto, uint16_t pin, bool invertido) {
    f->puerto        = puerto;
    f->pin           = pin;
    f->invertido     = invertido;
    f->flag          = false;
    f->ultimoDisparo = 0;
}

bool FinalDeCarrera_estaPresionado(FinalDeCarrera* f) {
    GPIO_PinState estado = HAL_GPIO_ReadPin(f->puerto, f->pin);
    return f->invertido ? (estado == GPIO_PIN_SET) : (estado == GPIO_PIN_RESET);
}

void FinalDeCarrera_onInterrupcion(FinalDeCarrera* f) {
    uint32_t ahora = HAL_GetTick();
    if ((ahora - f->ultimoDisparo) >= DEBOUNCE_MS && FinalDeCarrera_estaPresionado(f)) {
        f->ultimoDisparo = ahora;
        f->flag = true;
    }
}

bool FinalDeCarrera_getFlag(FinalDeCarrera* f) {
    return f->flag;
}

void FinalDeCarrera_resetFlag(FinalDeCarrera* f) {
    f->flag = false;
}

uint16_t FinalDeCarrera_getPin(FinalDeCarrera* f) {
    return f->pin;
}
