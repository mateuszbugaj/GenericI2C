#pragma once
#include <I2C_HAL.h>

struct HAL_Pin {
  uint32_t portRegister;
  uint16_t pinNumber;
  HAL_PullupConfig pullup;
  HAL_PinDirection direction;
  HAL_PinLevel level;
};

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint32_t portRegister, uint16_t pinNumber);