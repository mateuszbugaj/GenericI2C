#pragma once
#include <stdint.h>

typedef enum {
    INPUT = 0,
    OUTPUT = 1
} HAL_PinDirection;

typedef enum {
    LOW = 0,
    HIGH = 1
} HAL_PinLevel;

typedef enum {
  PULLUP_DISABLE,
  PULLUP_ENABLE  
} HAL_PullupConfig;

typedef struct {
    uint16_t* port;
    uint8_t pin;
    HAL_PullupConfig pullup;
    HAL_PinDirection direction;
    HAL_PinLevel level;
} HAL_Pin;

#ifdef __cplusplus
extern "C" {
#endif

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint16_t* port, uint8_t pin, HAL_PullupConfig pullup);
void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction);
void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level);
HAL_PinLevel HAL_pinRead(HAL_Pin* pin);
void HAL_sleep(uint16_t ms);

#ifdef DESKTOP
void HAL_registerPin(HAL_Pin* pin, const char* name);
#endif

#ifdef __cplusplus
}
#endif