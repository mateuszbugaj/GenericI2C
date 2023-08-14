#ifndef I2C_H
#define I2C_H
#include <stdbool.h>
#include <avr/delay.h>
#include "hal.h"

enum I2C_Role {
  MASTER,
  SLAVE
};

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
  enum I2C_Role role;
  HALPin SCLPin;
  HALPin SDAPin;
} I2C_Config;

void I2C_init(I2C_Config* config);
void I2C_setPrintFunc(void (*ptr)(const char[]));
void I2C_setPrintNumFunc(void (*ptr)(const int));
void I2C_sendStartCondition(void);
void I2C_sendStopCondition(void);

#endif /* I2C_H */