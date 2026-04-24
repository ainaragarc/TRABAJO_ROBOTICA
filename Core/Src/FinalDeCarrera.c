#include "FinalDeCarrera.h"

FinalDeCarrera::FinalDeCarrera(GPIO_TypeDef* puerto, uint16_t pin, bool invertido)
    : _puerto(puerto), _pin(pin), _invertido(invertido), _flag(false), _ultimoDisparo(0) {}

bool FinalDeCarrera::estaPresionado() {
    GPIO_PinState estado = HAL_GPIO_ReadPin(_puerto, _pin);
    return _invertido ? (estado == GPIO_PIN_SET) : (estado == GPIO_PIN_RESET);
}

// Llamar desde HAL_GPIO_EXTI_Callback. Filtra rebotes mecánicos del microswitch.
void FinalDeCarrera::onInterrupcion() {
    uint32_t ahora = HAL_GetTick();
    if ((ahora - _ultimoDisparo) >= DEBOUNCE_MS && estaPresionado()) {
        _ultimoDisparo = ahora;
        _flag = true;
    }
}

bool FinalDeCarrera::getFlag() {
    return _flag;
}

void FinalDeCarrera::resetFlag() {
    _flag = false;
}

uint16_t FinalDeCarrera::getPin() const {
    return _pin;
}
