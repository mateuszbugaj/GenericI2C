#pragma once
#include <avr/io.h>
#include <util/delay.h>
#include <I2C_HAL.h>

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint16_t* port, uint8_t pin, HAL_PullupConfig pullup){
  newPin->port = port;
  newPin->pin = pin;
  newPin->pullup = pullup;
  newPin->direction = OUTPUT;
  newPin->level = LOW;
  return newPin;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  if (direction == OUTPUT) {
    *(pin->port - 1) |= (1 << pin->pin); // DDRx is one address below PORTx
  } else {
    *(pin->port - 1) &= ~(1 << pin->pin);
    if(pin->pullup == PULLUP_ENABLE){
      *(pin->port - 2) |= (1 << pin->pin);
    }
  }
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level){
  if (level == HIGH) {
    *(pin->port) |= (1 << pin->pin);
  } else {
    *(pin->port) &= ~(1 << pin->pin);
  }
}

HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  return (*(pin->port - 2) & (1 << pin->pin)) ? HIGH : LOW; // PINx is two addresses below PORTx
}

void HAL_sleep(uint16_t ms){
  for(uint16_t i = 0; i < ms; i++){
    _delay_ms(1);
  }
}