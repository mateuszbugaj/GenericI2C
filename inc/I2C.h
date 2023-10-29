#pragma once
#include <stdbool.h>

#include <I2C_HAL.h>

typedef enum {
  MASTER,
  SLAVE
} I2C_Role;

typedef enum {
  WRITE, // (0B) indicates transmission
  READ // (1B)indicates request for data
} I2C_Data_Direction;

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
  I2C_Role role;
  HAL_Pin* sclOutPin;
  HAL_Pin* sdaOutPin;
  HAL_Pin* sclInPin;
  HAL_Pin* sdaInPin;
  uint32_t timeUnit;
  void (*print_str)(char[]);
  void (*print_num)(uint16_t);
} I2C_Config;

void I2C_init(I2C_Config* config);
void I2C_sendStartCondition(void);
void I2C_sendRepeatedStartCondition(void);
void I2C_sendStopCondition(void);
void I2C_read(void);
uint8_t I2C_receive(bool ack);
bool I2C_write(uint8_t payload);
bool I2C_writeAddress(uint8_t address, I2C_Data_Direction direction);
uint8_t I2C_lastByte();
bool I2C_newByteReceived(bool consume);