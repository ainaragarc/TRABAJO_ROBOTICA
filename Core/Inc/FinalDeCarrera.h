#ifndef FINAL_CARRERA_H
#define FINAL_CARRERA_H

#include "main.h"

class FinalDeCarrera {
private:
    GPIO_TypeDef* _puerto;
    uint16_t _pin;
    bool _invertido;
    volatile bool _flag;
    uint32_t _ultimoDisparo;
    static constexpr uint32_t DEBOUNCE_MS = 20;

public:
    FinalDeCarrera(GPIO_TypeDef* puerto, uint16_t pin, bool invertido = false);
    bool estaPresionado();      // lectura directa del pin
    void onInterrupcion();      // llamar desde HAL_GPIO_EXTI_Callback
    bool getFlag();             // true si se disparó por interrupción
    void resetFlag();
    uint16_t getPin() const;
};

#endif
