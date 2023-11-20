#pragma once
#include <I2C_HAL.h>

struct HAL_Pin {
  uint8_t* portRegister;
  uint8_t* pinRegister;
  uint8_t* ddRegister;
  uint8_t pinNumber;
  HAL_PullupConfig pullup;
  HAL_PinDirection direction;
  HAL_PinLevel level;
};

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint8_t* portRegister, uint8_t* pinRegister, uint8_t pinNumber, uint8_t* ddRegister);