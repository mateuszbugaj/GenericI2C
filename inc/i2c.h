#ifndef I2C_H
#define I2C_H
#include <stdbool.h>
#include <avr/delay.h>
#include "hal.h"

typedef enum I2C_Role {
  MASTER,
  SLAVE
};

typedef enum I2C_Data_Direction {
  WRITE, // (0B) indicates transmission
  READ // (1B)indicates request for data
};

/*
  Logging level:
  0 - Nothing
  1 - Role, Address
  2 - Sending/received bytes, is addressed, transmission END
  3 - START/STOP condition, ACK transmission, detected byte
  4 - Each received and sent bit
*/
typedef struct {
  uint8_t addr;
  bool respondToGeneralCall;
  uint8_t loggingLevel;
  enum I2C_Role role;
  HALPin sclOutPin;
  HALPin sdaOutPin;
  HALPin sclInPin;
  HALPin sdaInPin;
  uint8_t timeUnit;
} I2C_Config;

void I2C_init(I2C_Config* config);
void I2C_setPrintFunc(void (*ptr)(const char[]));
void I2C_setPrintNumFunc(void (*ptr)(const int));
void I2C_sendStartCondition(void);
void I2C_sendRepeatedStartCondition(void);
void I2C_sendStopCondition(void);
void I2C_read(void);
uint8_t I2C_receive(bool ack);
bool I2C_write(uint8_t payload);
bool I2C_writeAddress(uint8_t address, enum I2C_Data_Direction direction);
uint8_t I2C_lastByte();
bool I2C_newByteReceived(bool consume);

#endif /* I2C_H */