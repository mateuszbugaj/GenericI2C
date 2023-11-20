#include <I2C.h>

#define TIME_UNIT_LIMIT 15 // (the shortest allowed time unit in ms)
#define BYTE_SIZE 8

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

typedef struct {
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
} I2C_Internal_Config;

/*
  Pointer to printing function. Used to print logs for example via USART interface.
  By default, an empty function.
*/
void defaultPrint(char chars[]){}
void defaultPrintNum(uint16_t value){}
void (*printFuncPtr)(char content[]) = &defaultPrint;
void (*printNumFuncPtr)(uint16_t value) = &defaultPrintNum;

I2C_Config* cfg;
I2C_Internal_Config intCfg;

void I2C_log(char* content, uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(content);
    (*printFuncPtr)("\n\r");
  }
}

void I2C_logNum(char name[], uint16_t value, uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(name);
    (*printFuncPtr)(": ");
    (*printNumFuncPtr)(value);
    (*printFuncPtr)("\n\r");
  }
}

void releasePin(HAL_Pin* pin){
  HAL_pinWrite(pin, LOW);
}

void pullDownPin(HAL_Pin* pin){
  HAL_pinWrite(pin, HIGH);
}

void I2C_init(I2C_Config* config){
  cfg = config;

  if(cfg->print_str != ((void *)0)){
    printFuncPtr = cfg->print_str;
    printNumFuncPtr = cfg->print_num;
  } else {
    cfg->loggingLevel = 0;
  }

  I2C_log("I2C Start", 0);
  I2C_log(cfg->role==MASTER?"MASTER":"SLAVE", 1);

  if(cfg->addr == 0 || cfg->addr > 127){
    I2C_logNum("I2C addr bad", cfg->addr, 1);
    return;
  }

  if(cfg->timeUnit < TIME_UNIT_LIMIT){
    I2C_logNum("I2C TU short", cfg->timeUnit, 1);
    return;
  }

  I2C_logNum("Addr", cfg->addr, 1);

  HAL_setPinDirection(cfg->sdaOutPin, OUTPUT);
  HAL_setPinDirection(cfg->sclOutPin, OUTPUT);
  HAL_setPinDirection(cfg->sclInPin, INPUT);
  HAL_setPinDirection(cfg->sdaInPin, INPUT);

  intCfg.state = LISTENING_FOR_START;
  intCfg.sclPinLevel = HIGH;
  intCfg.sdaPinLevel = HIGH;
  intCfg.dataDirection = WRITE;
  intCfg.newByteReceived = false;
  intCfg.byteToSend = 0;
  releasePin(cfg->sclOutPin);
  releasePin(cfg->sdaOutPin);
}

void wait() {
  HAL_sleep(cfg->timeUnit);
}

void waitShort(){
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

void logState(uint8_t state){
  I2C_logNum("State", state, 2);
}

void I2C_read(){
  HAL_PinLevel newSclLevel = HAL_pinRead(cfg->sclInPin);
  HAL_PinLevel newSdaLevel = HAL_pinRead(cfg->sdaInPin);

  /* Clock events (SCL falling edge) */
  if(intCfg.sclPinLevel == HIGH && newSclLevel == LOW){
    switch (intCfg.state){
    case START:
    /* First SCL impulse after START is used for state change to READING_ADDRESS */
    intCfg.state = READING_ADDRESS;
    logState(intCfg.state);
    break;

    case READING_ADDRESS:
    intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum("Rx bit", intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter], 4);
    intCfg.DSRCounter++;

    if(intCfg.DSRCounter == BYTE_SIZE){
      intCfg.receivedByte = binaryToDecimal(intCfg.DSR);
      intCfg.DSRCounter = 0;
      uint8_t addr = intCfg.receivedByte >> 1;
      I2C_Data_Direction direction = intCfg.receivedByte & 0x01 ? READ : WRITE;

      I2C_logNum("Rx addr", addr, 3);

      if(addr == cfg->addr){
        I2C_log("Addr OK", 2);

        if(direction == WRITE){
          intCfg.dataDirection = WRITE;
        } else {
          intCfg.dataDirection = READ;
        }
        I2C_logNum("Dir", intCfg.dataDirection, 3);

        pullDownPin(cfg->sdaOutPin);
        intCfg.state = ACK_ADDRESS;
        logState(intCfg.state);
      } else {
        intCfg.state = LISTENING_FOR_START;
        logState(intCfg.state);
      }
    }
    break;

    case ACK_ADDRESS:
    /* First SCL impulse after READING_ADDRESS is to stop sending ACK (release SDA) and change state to RE_START */
    releasePin(cfg->sdaOutPin);

    if(intCfg.dataDirection == WRITE){
      intCfg.state = READING_PAYLOAD;
      logState(intCfg.state);
    } else {
      intCfg.state = RESPONDING_WITH_PAYLOAD;
      logState(intCfg.state);

      uint8_t bitToSend = intCfg.byteToSendArr[BYTE_SIZE - 1 - intCfg.byteToSendCounter];
      I2C_logNum("Tx bit", bitToSend, 4);
      if(bitToSend == 1){
        releasePin(cfg->sdaOutPin);
      } else {
        pullDownPin(cfg->sdaOutPin);
      }

      intCfg.byteToSendCounter++;
    }
    break;

    case READING_PAYLOAD:
    intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum("Rx bit", intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter], 4);
    intCfg.DSRCounter++;

    if(intCfg.DSRCounter == BYTE_SIZE){
      intCfg.receivedByte = binaryToDecimal(intCfg.DSR);
      intCfg.newByteReceived = true;
      I2C_logNum("Rx", intCfg.receivedByte, 2);
      pullDownPin(cfg->sdaOutPin);
      intCfg.state = ACK_PAYLOAD;
      logState(intCfg.state);
    }
    break;

    case RESPONDING_WITH_PAYLOAD:
    ;
    uint8_t bitToSend = intCfg.byteToSendArr[BYTE_SIZE - 1 - intCfg.byteToSendCounter];
    I2C_logNum("Tx bit", bitToSend, 4);
    if(bitToSend == 1){
      releasePin(cfg->sdaOutPin);
    } else {
      pullDownPin(cfg->sdaOutPin);
    }

    intCfg.byteToSendCounter++;

    if(intCfg.byteToSendCounter == BYTE_SIZE + 1){
      intCfg.byteToSendCounter = 0;
      releasePin(cfg->sdaOutPin);
      intCfg.state = READING_ACK;
      logState(intCfg.state);
    }
    break;

    case READING_ACK:
    ;
    HAL_PinLevel ack = HAL_pinRead(cfg->sdaInPin);
    if(ack == LOW){
      I2C_log("RX ACK", 3);
    } else {
      I2C_log("RX N-ACK", 3);
    }

    intCfg.state = READING_PAYLOAD;
    logState(intCfg.state);
    break;

    case ACK_PAYLOAD:
    /* First SCL impulse after READING_PAYLOAD is to stop sending ACK (release SDA) and change state to LISTENING_FOR_STOP */
    releasePin(cfg->sdaOutPin);
    intCfg.DSRCounter = 0;
    intCfg.state = READING_PAYLOAD;
    logState(intCfg.state);
    break;

    default:
    break;
    }
  }

  /* START and STOP conditions (SDA change during SCL HIGH) */
  if(newSclLevel == HIGH){

    /* SDA falling edge (START conditions) */
    if(intCfg.sdaPinLevel == HIGH && newSdaLevel == LOW){
      I2C_log("START", 3);
      if(intCfg.state == LISTENING_FOR_START || intCfg.state == READING_PAYLOAD){
        intCfg.DSRCounter = 0;
        intCfg.byteToSendCounter = 0;
        intCfg.dataDirection = WRITE;
        intCfg.state = START;
        logState(intCfg.state);
      }
    }

    /* SDA raising edge (STOP conditions) */
    if(intCfg.sdaPinLevel == LOW && newSdaLevel == HIGH){
      I2C_log("STOP", 3);
      if(intCfg.state == READING_PAYLOAD){
        if(intCfg.DSRCounter == 0){
          I2C_log("TX END", 2);
        } else {
          I2C_log("Unexpected STOP", 2);
        }
      }

      intCfg.state = LISTENING_FOR_START;
      logState(intCfg.state);
      intCfg.DSRCounter = 0;
    }
  }

  intCfg.sclPinLevel = newSclLevel;
  intCfg.sdaPinLevel = newSdaLevel;
}

uint8_t I2C_receive(bool ack){
    uint8_t receivedBits[8];
    for(uint8_t i = BYTE_SIZE - 1; i != 255; i--){
      I2C_logNum("Rx bit nr", i, 4);
      releasePin(cfg->sclOutPin);
      wait();
      HAL_PinLevel result = HAL_pinRead(cfg->sdaInPin);
      if(result == HIGH){
        receivedBits[i] = 1;
      } else {
        receivedBits[i] = 0;
      }
      I2C_logNum("Rx bit", receivedBits[i], 4);
      pullDownPin(cfg->sclOutPin);
      wait();
    }

    uint8_t response = binaryToDecimal(receivedBits);
    I2C_logNum("RX", response, 2);
    I2C_log("TX ACK", 3);

    // Respond with ACK or NACK
    if(ack){
      pullDownPin(cfg->sdaOutPin);
      waitShort();
      releasePin(cfg->sclOutPin);
      wait();
      pullDownPin(cfg->sclOutPin);
      waitShort();
      releasePin(cfg->sdaOutPin);
    } else {
      releasePin(cfg->sclOutPin);
      wait();
      pullDownPin(cfg->sclOutPin);
      wait();
    }

    wait();
    releasePin(cfg->sclOutPin);

    I2C_log("ACK TX STOP", 4);

    return response;
}

bool I2C_write(uint8_t payload){
  // TODO: check if the line is not busy
  I2C_logNum("TX", payload, 2);
  if(cfg->role == SLAVE){
    decimalToBinary(payload, intCfg.byteToSendArr);
    return true;
  }

  decimalToBinary(payload, intCfg.DSR);

  pullDownPin(cfg->sdaOutPin);
  wait();
  for(uint8_t i = BYTE_SIZE - 1; i != 255; i--){
    I2C_logNum("TX bit", intCfg.DSR[i], 4);
    if(intCfg.DSR[i] == 0){
      releasePin(cfg->sclOutPin);
      wait();
      pullDownPin(cfg->sclOutPin);
      wait();
    } else {
      releasePin(cfg->sdaOutPin);
      waitShort();
      releasePin(cfg->sclOutPin);
      wait();
      pullDownPin(cfg->sclOutPin);
      waitShort();
      pullDownPin(cfg->sdaOutPin);
      wait();
    }
  }

  // Read ACK
  releasePin(cfg->sdaOutPin);
  wait();
  releasePin(cfg->sclOutPin);
  wait();
  bool ack = HAL_pinRead(cfg->sdaInPin) == LOW ? true : false;
  pullDownPin(cfg->sclOutPin);
  // pullDownPin(cfg->sdaOutPin);
  wait();

  if(ack){
    I2C_log("RX ACK", 3);
  } else {
    I2C_log("RX N-ACK", 3);
  }
  return ack;
}

bool I2C_writeAddress(uint8_t address, I2C_Data_Direction direction){
  uint8_t addr = (address << 1); // shift left to make space for R/W bit

  if(direction == READ){
    addr |= 0x01; // Set the R/W bit to 1 for READ
  }

  return I2C_write(addr);
}

uint8_t I2C_lastByte(){
  if(cfg->role == SLAVE){
    return intCfg.receivedByte;
  }
  return 0;
}

bool I2C_newByteReceived(bool consume){
  if(intCfg.newByteReceived){
    if(consume){
      intCfg.newByteReceived = false;
    } 

    return true;
  }

  return false;
}

/* Change of SDA HIGH -> LOW with SCL HIGH */
void I2C_sendStartCondition() {
  I2C_log("TX START", 3);
  pullDownPin(cfg->sdaOutPin);
  wait();
  pullDownPin(cfg->sclOutPin);
  wait();
}

/* 
  Change of SDA LOW -> HIGH with SCL LOW 
  Change of SDA HIGH -> LOW with SCL HIGH 
*/
void I2C_sendRepeatedStartCondition() {
  I2C_log("TX RE-START", 3);
  pullDownPin(cfg->sdaOutPin);
  waitShort();
  pullDownPin(cfg->sclOutPin);
  waitShort();
  releasePin(cfg->sdaOutPin);
  wait();
  releasePin(cfg->sclOutPin);
  wait();
  pullDownPin(cfg->sdaOutPin);
  wait();
  pullDownPin(cfg->sclOutPin);
  waitShort();
}

/* Change of SDA LOW -> HIGH with SCL HIGH */
void I2C_sendStopCondition() {
  I2C_log("TX STOP", 3);
  pullDownPin(cfg->sdaOutPin);
  wait();
  releasePin(cfg->sclOutPin);
  wait();
  releasePin(cfg->sdaOutPin);
  wait();
}