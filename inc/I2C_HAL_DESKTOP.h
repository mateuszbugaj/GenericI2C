#pragma once
#include <I2C_HAL.h>

typedef enum {
  SCL_IN,
  SCL_OUT,
  SDA_IN,
  SDA_OUT,
  SCL,
  SDA
} HAL_PinRole;

struct HAL_Pin {
  uint16_t pinNumber;
  HAL_PullupConfig pullup;
  HAL_PinDirection direction;
  HAL_PinLevel level;
  HAL_PinRole pinRole;
};

HAL_Pin* HAL_pinSetup(HAL_PinRole pinRole);
HAL_Pin* HAL_SclPin();
HAL_Pin* HAL_SdaPin();