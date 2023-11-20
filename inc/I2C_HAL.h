#pragma once
#include <stdint.h>

typedef enum {
    OUTPUT = 0,
    INPUT = 1
} HAL_PinDirection;

typedef enum {
    LOW = 0,
    HIGH = 1
} HAL_PinLevel;

typedef enum {
  PULLUP_DISABLE,
  PULLUP_ENABLE  
} HAL_PullupConfig;

typedef struct HAL_Pin HAL_Pin; // Forward declaration

#ifdef __cplusplus
extern "C" {
#endif

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction);
void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level);
HAL_PinLevel HAL_pinRead(HAL_Pin* pin);
void HAL_sleep(uint16_t ms);

#ifdef __cplusplus
}
#endif