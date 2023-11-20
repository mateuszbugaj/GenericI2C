#pragma once
#include <avr/io.h>
#include <util/delay.h>
#include <I2C_HAL.h>
#include <I2C_HAL_AVR.h>

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint8_t* portRegister, uint8_t* pinRegister, uint8_t pinNumber, uint8_t* ddRegister){
  newPin->portRegister = portRegister;
  newPin->pinRegister = pinRegister;
  newPin->pinNumber = pinNumber;
  newPin->ddRegister = ddRegister;
  return newPin;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  if (direction == OUTPUT) {
    *(pin->ddRegister) |= (1 << pin->pinNumber);
  } else {
    *(pin->ddRegister) &= ~(1 << pin->pinNumber);
    *(pin->portRegister) |= (1 << pin->pinNumber);
  }
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level){
  if (level == HIGH) {
    *(pin->portRegister) |= (1 << pin->pinNumber);
  } else {
    *(pin->portRegister) &= ~(1 << pin->pinNumber);
  }
}

HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  return (*(pin->pinRegister) & (1 << pin->pinNumber)) ? HIGH : LOW;
}

void HAL_sleep(uint16_t ms){
  for(uint16_t i = 0; i < ms; i++){
    _delay_ms(1);
  }
}