#ifndef I2C_H
#define I2C_H
#include <stdbool.h>
#include "hal.h"

/*
  Logging level:
  0 - Nothing
  1 - Role, Address
  2 - Sending/received bytes, is addressed, transmission END
  3 - START/STOP condition, ACK transmission, detected byte
  4 - Each received bit
*/
typedef struct {
  uint8_t addr;
  bool respondToGeneralCall;
  uint8_t loggingLevel;
} I2C_Config;

void I2C_init(I2C_Config* config);
void I2C_setPrintFunc(void (*ptr)(const char[], uint8_t, ...));

#endif /* I2C_H */