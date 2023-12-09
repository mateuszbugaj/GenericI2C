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

typedef enum {
  LISTENING_FOR_START,
  START,
  READING_ADDRESS,
  ACK_ADDRESS,
  ADDRESSED,
  READING_PAYLOAD,
  ACK_PAYLOAD,
  RESPONDING_WITH_PAYLOAD,
  READING_ACK
} State;

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
  void (*print_str)(I2C_Role, char[]);
  void (*print_num)(I2C_Role, uint16_t);
  State state;
  HAL_PinLevel sclPinLevel;
  HAL_PinLevel sdaPinLevel;
  uint8_t DSR[8];        // Data Shift Register - contains bits which are currently being downloaded from the I2C bus.
  uint8_t DSRCounter;    // Holds bit number in currently downloaded or uploaded byte. It can have value from 0 to 9 (this includes ACK impulse).
  uint8_t receivedByte;  // Holds last whole byte decoted to decimal from DSR.
  bool newByteReceived;  // Used for pooling event
  I2C_Data_Direction dataDirection;
  uint8_t byteToSend;    // Byte to be sent when master asks for data
  uint8_t byteToSendCounter;
  uint8_t byteToSendArr[8];
} I2C_Config;

void I2C_init(I2C_Config* config);
void I2C_sendStartCondition(I2C_Config* config);
void I2C_sendRepeatedStartCondition(I2C_Config* config);
void I2C_sendStopCondition(I2C_Config* config);
void I2C_read(I2C_Config* config);
uint8_t I2C_receive(I2C_Config* config, bool ack);
bool I2C_write(I2C_Config* config, uint8_t payload);
bool I2C_writeAddress(I2C_Config* config, uint8_t address, I2C_Data_Direction direction);
uint8_t I2C_lastByte(I2C_Config* config);
bool I2C_newByteReceived(I2C_Config* config, bool consume);