#include <I2C.h>

#define TIME_UNIT_LIMIT 15 // (the shortest allowed time unit in ms)
#define BYTE_SIZE 8

/*
  Pointer to printing function. Used to print logs for example via USART interface.
  By default, an empty function.
*/
void defaultPrint(I2C_Role role, char chars[]){}
void defaultPrintNum(I2C_Role role, uint16_t value){}
void (*printFuncPtr)(I2C_Role role, char content[]) = &defaultPrint;
void (*printNumFuncPtr)(I2C_Role role, uint16_t value) = &defaultPrintNum;

void I2C_log(I2C_Config* cfg, char* content, uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(cfg->role, content);
    (*printFuncPtr)(cfg->role, "\n\r");
  }
}

void I2C_logNum(I2C_Config* cfg, char name[], uint16_t value, uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(cfg->role, name);
    (*printFuncPtr)(cfg->role, ": ");
    (*printNumFuncPtr)(cfg->role, value);
    (*printFuncPtr)(cfg->role, "\n\r");
  }
}

void releasePin(HAL_Pin* pin){
  HAL_pinWrite(pin, LOW);
}

void pullDownPin(HAL_Pin* pin){
  HAL_pinWrite(pin, HIGH);
}

void I2C_init(I2C_Config* cfg){
  if(cfg->print_str != ((void *)0)){
    printFuncPtr = cfg->print_str;
    printNumFuncPtr = cfg->print_num;
  } else {
    cfg->loggingLevel = 0;
  }

  I2C_log(cfg, "I2C Start", 0);
  I2C_log(cfg, cfg->role==MASTER?"MASTER":"SLAVE", 1);

  if(cfg->addr == 0 || cfg->addr > 127){
    I2C_logNum(cfg, "I2C addr bad", cfg->addr, 1);
    return;
  }

  if(cfg->timeUnit < TIME_UNIT_LIMIT){
    I2C_logNum(cfg, "I2C TU short", cfg->timeUnit, 1);
    return;
  }

  I2C_logNum(cfg, "Addr", cfg->addr, 1);

  HAL_setPinDirection(cfg->sdaOutPin, OUTPUT);
  HAL_setPinDirection(cfg->sclOutPin, OUTPUT);
  HAL_setPinDirection(cfg->sclInPin, INPUT);
  HAL_setPinDirection(cfg->sdaInPin, INPUT);

  cfg->state = LISTENING_FOR_START;
  cfg->sclPinLevel = HIGH;
  cfg->sdaPinLevel = HIGH;
  cfg->dataDirection = WRITE;
  cfg->newByteReceived = false;
  cfg->byteToSend = 0;
  releasePin(cfg->sclOutPin);
  releasePin(cfg->sdaOutPin);
}

void wait(I2C_Config* cfg) {
  HAL_sleep(cfg->timeUnit);
}

void waitShort(I2C_Config* cfg){
  HAL_sleep(cfg->timeUnit/4);
}

void decimalToBinary(uint8_t byte, uint8_t* arr) {
    uint8_t rem = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < BYTE_SIZE; i++) {
        arr[i] = 0;
    }

    while (byte != 0) {
        rem = byte % 2;
        byte /= 2;
        arr[count] = rem;
        count++;
        if (count > 7) {
            break;
        }
    }
}

uint8_t binaryToDecimal(uint8_t* arr) {
    uint8_t value = 0;
    uint8_t multiplier = 1;
    for (uint8_t i = 0; i < BYTE_SIZE; i++) {
        value += multiplier * arr[i];
        multiplier *= 2;
    }

    return value;
}

/*
A pooling function for slaves to listen for incoming bits and conditions implemented as a state machine.

|                        ADDRESS TRANSMISSION                                    PAYLOAD TRANSMISSION                          |
|                    |      x8      |   ACK(LOW)  |                         |      x8      |     ACK     |                     |
|________            | ____________ | ___________ |  _____                  | ____________ | ___________ |  _____            __|
| SDA    \____________/____________\_/___________\__/     \__________________/____________\_/___________\__/     \__________/  |
|.       .    .                  .              .         .          .                 .               .                    .  |
|.       .    .                  .              .         .          .                 .               .                    .  |
|______________          ________.     ________ .      ______________          ________.      ________ .              _________|
|.SCL    .     \________/        \____/        \______/   .          \________/        \_____/        \______________/      .  |
|.       .    .                  .              .         .          .                 .               .                    .  |
|.STATE  .    .                  .              .         .          .                 .               .                    .  |
|.       .    +READING_ADDRESS   .              .         +RE_START  .                 .               . LISTENING_FOR_START+  |
|.       +START                  .              -(release SDA)       .                 .               +READING_PAYLOAD        |
|+LISTENING_FOR_START            +ACK_ADDRESS                        +READING_PAYLOAD  +ACK_PAYLOAD    -(release SDA)          |
*/

void logState(I2C_Config* cfg, uint8_t state){
  I2C_logNum(cfg, "State", state, 2);
}

void I2C_read(I2C_Config* cfg){
  HAL_PinLevel newSclLevel = HAL_pinRead(cfg->sclInPin);
  HAL_PinLevel newSdaLevel = HAL_pinRead(cfg->sdaInPin);

  /* Clock events (SCL falling edge) */
  if(cfg->sclPinLevel == HIGH && newSclLevel == LOW){
    switch (cfg->state){
    case START:
    /* First SCL impulse after START is used for state change to READING_ADDRESS */
    cfg->state = READING_ADDRESS;
    logState(cfg, cfg->state);
    break;

    case READING_ADDRESS:
    cfg->DSR[BYTE_SIZE - 1 - cfg->DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum(cfg, "Rx bit", cfg->DSR[BYTE_SIZE - 1 - cfg->DSRCounter], 4);
    cfg->DSRCounter++;

    if(cfg->DSRCounter == BYTE_SIZE){
      cfg->receivedByte = binaryToDecimal(cfg->DSR);
      cfg->DSRCounter = 0;
      uint8_t addr = cfg->receivedByte >> 1;
      I2C_Data_Direction direction = cfg->receivedByte & 0x01 ? READ : WRITE;

      I2C_logNum(cfg, "Rx addr", addr, 3);

      if(addr == cfg->addr){
        I2C_log(cfg, "Addr OK", 2);

        if(direction == WRITE){
          cfg->dataDirection = WRITE;
        } else {
          cfg->dataDirection = READ;
        }
        I2C_logNum(cfg, "Dir", cfg->dataDirection, 3);

        pullDownPin(cfg->sdaOutPin);
        cfg->state = ACK_ADDRESS;
        logState(cfg, cfg->state);
      } else {
        cfg->state = LISTENING_FOR_START;
        logState(cfg, cfg->state);
      }
    }
    break;

    case ACK_ADDRESS:
    /* First SCL impulse after READING_ADDRESS is to stop sending ACK (release SDA) and change state to RE_START */
    releasePin(cfg->sdaOutPin);

    if(cfg->dataDirection == WRITE){
      cfg->state = READING_PAYLOAD;
      logState(cfg, cfg->state);
    } else {
      cfg->state = RESPONDING_WITH_PAYLOAD;
      logState(cfg, cfg->state);

      uint8_t bitToSend = cfg->byteToSendArr[BYTE_SIZE - 1 - cfg->byteToSendCounter];
      I2C_logNum(cfg, "Tx bit", bitToSend, 4);
      if(bitToSend == 1){
        releasePin(cfg->sdaOutPin);
      } else {
        pullDownPin(cfg->sdaOutPin);
      }

      cfg->byteToSendCounter++;
    }
    break;

    case READING_PAYLOAD:
    cfg->DSR[BYTE_SIZE - 1 - cfg->DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum(cfg, "Rx bit", cfg->DSR[BYTE_SIZE - 1 - cfg->DSRCounter], 4);
    cfg->DSRCounter++;

    if(cfg->DSRCounter == BYTE_SIZE){
      cfg->receivedByte = binaryToDecimal(cfg->DSR);
      cfg->newByteReceived = true;
      I2C_logNum(cfg, "Rx", cfg->receivedByte, 2);
      pullDownPin(cfg->sdaOutPin);
      cfg->state = ACK_PAYLOAD;
      logState(cfg, cfg->state);
    }
    break;

    case RESPONDING_WITH_PAYLOAD:
    ;
    uint8_t bitToSend = cfg->byteToSendArr[BYTE_SIZE - 1 - cfg->byteToSendCounter];
    I2C_logNum(cfg, "Tx bit", bitToSend, 4);
    if(bitToSend == 1){
      releasePin(cfg->sdaOutPin);
    } else {
      pullDownPin(cfg->sdaOutPin);
    }

    cfg->byteToSendCounter++;

    if(cfg->byteToSendCounter == BYTE_SIZE + 1){
      cfg->byteToSendCounter = 0;
      releasePin(cfg->sdaOutPin);
      cfg->state = READING_ACK;
      logState(cfg, cfg->state);
    }
    break;

    case READING_ACK:
    ;
    HAL_PinLevel ack = HAL_pinRead(cfg->sdaInPin);
    if(ack == LOW){
      I2C_log(cfg, "RX ACK", 3);
    } else {
      I2C_log(cfg, "RX N-ACK", 3);
    }

    cfg->state = READING_PAYLOAD;
    logState(cfg, cfg->state);
    break;

    case ACK_PAYLOAD:
    /* First SCL impulse after READING_PAYLOAD is to stop sending ACK (release SDA) and change state to LISTENING_FOR_STOP */
    releasePin(cfg->sdaOutPin);
    cfg->DSRCounter = 0;
    cfg->state = READING_PAYLOAD;
    logState(cfg, cfg->state);
    break;

    default:
    break;
    }
  }

  /* START and STOP conditions (SDA change during SCL HIGH) */
  if(newSclLevel == HIGH){

    /* SDA falling edge (START conditions) */
    if(cfg->sdaPinLevel == HIGH && newSdaLevel == LOW){
      I2C_log(cfg, "START", 3);
      if(cfg->state == LISTENING_FOR_START || cfg->state == READING_PAYLOAD){
        cfg->DSRCounter = 0;
        cfg->byteToSendCounter = 0;
        cfg->dataDirection = WRITE;
        cfg->state = START;
        logState(cfg, cfg->state);
      }
    }

    /* SDA raising edge (STOP conditions) */
    if(cfg->sdaPinLevel == LOW && newSdaLevel == HIGH){
      I2C_log(cfg, "STOP", 3);
      if(cfg->state == READING_PAYLOAD){
        if(cfg->DSRCounter == 0){
          I2C_log(cfg, "TX END", 2);
        } else {
          I2C_log(cfg, "Unexpected STOP", 2);
        }
      }

      cfg->state = LISTENING_FOR_START;
      logState(cfg, cfg->state);
      cfg->DSRCounter = 0;
    }
  }

  cfg->sclPinLevel = newSclLevel;
  cfg->sdaPinLevel = newSdaLevel;
}

uint8_t I2C_receive(I2C_Config* cfg, bool ack){
    uint8_t receivedBits[8];
    for(uint8_t i = BYTE_SIZE - 1; i != 255; i--){
      I2C_logNum(cfg, "Rx bit nr", i, 4);
      releasePin(cfg->sclOutPin);
      wait(cfg);
      HAL_PinLevel result = HAL_pinRead(cfg->sdaInPin);
      if(result == HIGH){
        receivedBits[i] = 1;
      } else {
        receivedBits[i] = 0;
      }
      I2C_logNum(cfg, "Rx bit", receivedBits[i], 4);
      pullDownPin(cfg->sclOutPin);
      wait(cfg);
    }

    uint8_t response = binaryToDecimal(receivedBits);
    I2C_logNum(cfg, "RX", response, 2);
    I2C_log(cfg, "TX ACK", 3);

    // Respond with ACK or NACK
    if(ack){
      pullDownPin(cfg->sdaOutPin);
      waitShort(cfg);
      releasePin(cfg->sclOutPin);
      wait(cfg);
      pullDownPin(cfg->sclOutPin);
      waitShort(cfg);
      releasePin(cfg->sdaOutPin);
    } else {
      releasePin(cfg->sclOutPin);
      wait(cfg);
      pullDownPin(cfg->sclOutPin);
      wait(cfg);
    }

    wait(cfg);
    releasePin(cfg->sclOutPin);

    I2C_log(cfg, "ACK TX STOP", 4);

    return response;
}

bool I2C_write(I2C_Config* cfg, uint8_t payload){
  // TODO: check if the line is not busy
  I2C_logNum(cfg, "TX", payload, 2);
  if(cfg->role == SLAVE){
    decimalToBinary(payload, cfg->byteToSendArr);
    return true;
  }

  decimalToBinary(payload, cfg->DSR);

  pullDownPin(cfg->sdaOutPin);
  wait(cfg);
  for(uint8_t i = BYTE_SIZE - 1; i != 255; i--){
    I2C_logNum(cfg, "TX bit", cfg->DSR[i], 4);
    if(cfg->DSR[i] == 0){
      releasePin(cfg->sclOutPin);
      wait(cfg);
      pullDownPin(cfg->sclOutPin);
      wait(cfg);
    } else {
      releasePin(cfg->sdaOutPin);
      waitShort(cfg);
      releasePin(cfg->sclOutPin);
      wait(cfg);
      pullDownPin(cfg->sclOutPin);
      waitShort(cfg);
      pullDownPin(cfg->sdaOutPin);
      wait(cfg);
    }
  }

  // Read ACK
  releasePin(cfg->sdaOutPin);
  wait(cfg);
  releasePin(cfg->sclOutPin);
  wait(cfg);
  bool ack = HAL_pinRead(cfg->sdaInPin) == LOW ? true : false;
  pullDownPin(cfg->sclOutPin);
  // pullDownPin(cfg->sdaOutPin);
  wait(cfg);

  if(ack){
    I2C_log(cfg, "RX ACK", 3);
  } else {
    I2C_log(cfg, "RX N-ACK", 3);
  }
  return ack;
}

bool I2C_writeAddress(I2C_Config* cfg, uint8_t address, I2C_Data_Direction direction){
  uint8_t addr = (address << 1); // shift left to make space for R/W bit

  if(direction == READ){
    addr |= 0x01; // Set the R/W bit to 1 for READ
  }

  return I2C_write(cfg, addr);
}

uint8_t I2C_lastByte(I2C_Config* cfg){
  if(cfg->role == SLAVE){
    return cfg->receivedByte;
  }
  return 0;
}

bool I2C_newByteReceived(I2C_Config* cfg, bool consume){
  if(cfg->newByteReceived){
    if(consume){
      cfg->newByteReceived = false;
    } 

    return true;
  }

  return false;
}

/* Change of SDA HIGH -> LOW with SCL HIGH */
void I2C_sendStartCondition(I2C_Config* cfg) {
  I2C_log(cfg, "TX START", 3);
  pullDownPin(cfg->sdaOutPin);
  wait(cfg);
  pullDownPin(cfg->sclOutPin);
  wait(cfg);
}

/* 
  Change of SDA LOW -> HIGH with SCL LOW 
  Change of SDA HIGH -> LOW with SCL HIGH 
*/
void I2C_sendRepeatedStartCondition(I2C_Config* cfg) {
  I2C_log(cfg, "TX RE-START", 3);
  pullDownPin(cfg->sdaOutPin);
  waitShort(cfg);
  pullDownPin(cfg->sclOutPin);
  waitShort(cfg);
  releasePin(cfg->sdaOutPin);
  wait(cfg);
  releasePin(cfg->sclOutPin);
  wait(cfg);
  pullDownPin(cfg->sdaOutPin);
  wait(cfg);
  pullDownPin(cfg->sclOutPin);
  waitShort(cfg);
}

/* Change of SDA LOW -> HIGH with SCL HIGH */
void I2C_sendStopCondition(I2C_Config* cfg) {
  I2C_log(cfg, "TX STOP", 3);
  pullDownPin(cfg->sdaOutPin);
  wait(cfg);
  releasePin(cfg->sclOutPin);
  wait(cfg);
  releasePin(cfg->sdaOutPin);
  wait(cfg);
}