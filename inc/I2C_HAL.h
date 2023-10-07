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
    uint8_t* port;
    uint8_t pin;
    HAL_PullupConfig pullup;
    HAL_PinDirection direction;
    HAL_PinLevel level;
} HAL_Pin;

typedef void (*HAL_LoggingCallback)(HAL_Pin* pins, uint8_t size);

#ifdef __cplusplus
extern "C" {
#endif

HAL_Pin* pinSetup(HAL_Pin* newPin, uint8_t* port, uint8_t pin, HAL_PullupConfig pullup);
void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction);
void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level);
HAL_PinLevel HAL_pinRead(HAL_Pin* pin);
void HAL_setLoggingCallback(HAL_LoggingCallback logFunc);

#ifdef __cplusplus
}
#endif