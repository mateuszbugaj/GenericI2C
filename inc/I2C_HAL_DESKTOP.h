#pragma once
#include <I2C_HAL.h>

struct HAL_Pin {
  uint16_t pinNumber;
  HAL_PullupConfig pullup;
  HAL_PinDirection direction;
  HAL_PinLevel level;
};

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint8_t pinNumber);
void HAL_registerPin(HAL_Pin* pin, const char* name);