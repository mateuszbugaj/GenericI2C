#include "i2c.h"
#define TIME_UNIT_LIMIT 15 // (the shortest allowed time unit in ms)
#define BYTE_SIZE 8

typedef enum State {
  LISTENING_FOR_START,
  START,
  READING_ADDRESS,
  ACK_ADDRESS,
  ADDRESSED,
  READING_PAYLOAD,
  ACK_PAYLOAD,
  RESPONDING_WITH_PAYLOAD,
  READING_ACK
};

typedef struct {
  enum State state;
  PinLevel sclPinLevel;
  PinLevel sdaPinLevel;
  uint8_t DSR[8];        // Data Shift Register - contains bits which are currently being downloaded from the I2C bus.
  uint8_t DSRCounter;    // Holds bit number in currently downloaded or uploaded byte. It can have value from 0 to 9 (this includes ACK impulse).
  uint8_t receivedByte;  // Holds last whole byte decoted to decimal from DSR.
  bool newByteReceived;  // Used for pooling event
  enum I2C_Data_Direction dataDirection;
  uint8_t byteToSend;    // Byte to be sent when master asks for data
  uint8_t byteToSendCounter;
  uint8_t byteToSendArr[8];
} I2C_Internal_Config;

/*
  Pointer to printing function. Used to print logs for example via USART interface.
  By default, an empty function.
*/
void defaultPrint(const char chars[]){}
void defaultPrintNum(const int value ){}
void (*printFuncPtr)(char content[]) = &defaultPrint;
void (*printNumFuncPtr)(int value) = &defaultPrintNum;
void I2C_setPrintFunc(void (*ptr)(const char[])){ printFuncPtr = ptr; }
void I2C_setPrintNumFunc(void (*ptr)(const int)){ printNumFuncPtr = ptr; }

I2C_Config* cfg;
I2C_Internal_Config intCfg;

void I2C_log(char content[], uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(content);
    (*printFuncPtr)("\n\r");
  }
}

void I2C_logNum(char name[], int value, uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(name);
    (*printFuncPtr)(": ");
    (*printNumFuncPtr)(value);
    (*printFuncPtr)("\n\r");
  }
}

void I2C_init(I2C_Config* config){
  cfg = config;

  I2C_log("I2C Start", 0);
  I2C_log(cfg->role==MASTER?"MASTER":"SLAVE", 1);

  if(cfg->addr == 0 || cfg->addr > 127){
    I2C_logNum("I2C address incorrect: ", cfg->addr, 1);
    return;
  }

  if(cfg->timeUnit < TIME_UNIT_LIMIT){
    I2C_logNum("I2C time unit too short: ", cfg->timeUnit, 1);
    return;
  }

  I2C_logNum("Addr", cfg->addr, 1);

  hal_pin_direction(cfg->sclOutPin, OUTPUT);
  hal_pin_direction(cfg->sdaOutPin, OUTPUT);
  hal_pin_direction(cfg->sclInPin, INPUT);
  hal_pin_direction(cfg->sdaInPin, INPUT);

  intCfg.state = LISTENING_FOR_START;
  intCfg.sclPinLevel = HIGH;
  intCfg.sdaPinLevel = HIGH;
  intCfg.dataDirection = WRITE;
  intCfg.newByteReceived = false;
  intCfg.byteToSend = 0;
  releasePin(cfg->sclOutPin);
  releasePin(cfg->sdaOutPin);
}

void releasePin(HALPin pin){
  hal_pin_write(pin, LOW);
}

void pullDownPin(HALPin pin){
  hal_pin_write(pin, HIGH);
}

void wait() {
  for(uint8_t i = 0; i < cfg->timeUnit; i++){
    _delay_ms(1);
  }
}

void waitShort(){
  for(uint8_t i = 0; i < cfg->timeUnit/4; i++){
    _delay_ms(1);
  }
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

void I2C_read(){
  PinLevel newSclLevel = hal_pin_read(cfg->sclInPin);
  PinLevel newSdaLevel = hal_pin_read(cfg->sdaInPin);

  /* Clock events (SCL falling edge) */
  if(intCfg.sclPinLevel == HIGH && newSclLevel == LOW){
    switch (intCfg.state){
    case START:
    /* First SCL impulse after START is used for state change to READING_ADDRESS */
    intCfg.state = READING_ADDRESS;
    I2C_log("State: READING_ADDRESS", 4);
    break;

    case READING_ADDRESS:
    intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum("Received bit: ", intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter], 4);
    intCfg.DSRCounter++;

    if(intCfg.DSRCounter == BYTE_SIZE){
      intCfg.receivedByte = binaryToDecimal(intCfg.DSR);
      intCfg.DSRCounter = 0;
      uint8_t addr = intCfg.receivedByte & 0b01111111;
      enum I2C_Data_Direction direction = intCfg.receivedByte & 0b10000000;

      I2C_logNum("Received address: ", addr, 3);

      if(addr == cfg->addr){
        I2C_logNum("Address match: ", cfg->addr, 2);

        if(direction == WRITE){
          I2C_log("Data direction: WRITE", 3);
          intCfg.dataDirection = WRITE;
        } else {
          I2C_log("Data direction: READ", 3);
          intCfg.dataDirection = READ;
        }

        pullDownPin(cfg->sdaOutPin);
        intCfg.state = ACK_ADDRESS;
        I2C_log("State: ACK_ADDRESS", 4);
      } else {
        intCfg.state = LISTENING_FOR_START;
        I2C_log("State: LISTENING_FOR_START", 4);
      }
    }
    break;

    case ACK_ADDRESS:
    /* First SCL impulse after READING_ADDRESS is to stop sending ACK (release SDA) and change state to RE_START */
    releasePin(cfg->sdaOutPin);

    if(intCfg.dataDirection == WRITE){
      intCfg.state = READING_PAYLOAD;
      I2C_log("State: READING_PAYLOAD", 4);
    } else {
      intCfg.state = RESPONDING_WITH_PAYLOAD;
      I2C_log("State: RESPONDING_WITH_PAYLOAD", 4);

      uint8_t bitToSend = intCfg.byteToSendArr[BYTE_SIZE - 1 - intCfg.byteToSendCounter];
      I2C_logNum("Responding with bit: ", bitToSend, 4);
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
    I2C_logNum("Received bit: ", intCfg.DSR[BYTE_SIZE - 1 - intCfg.DSRCounter], 4);
    intCfg.DSRCounter++;

    if(intCfg.DSRCounter == BYTE_SIZE){
      intCfg.receivedByte = binaryToDecimal(intCfg.DSR);
      intCfg.newByteReceived = true;
      I2C_logNum("Received payload: ", intCfg.receivedByte, 2);
      pullDownPin(cfg->sdaOutPin);
      intCfg.state = ACK_PAYLOAD;
      I2C_log("State: ACK_PAYLOAD", 4);
    }
    break;

    case RESPONDING_WITH_PAYLOAD:
    ;
    uint8_t bitToSend = intCfg.byteToSendArr[BYTE_SIZE - 1 - intCfg.byteToSendCounter];
    I2C_logNum("Responding with bit: ", bitToSend, 4);
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
      I2C_log("State: READING_ACK", 4);
    }
    break;

    case READING_ACK:
    ;
    PinLevel ack = hal_pin_read(cfg->sdaInPin);
    if(ack == LOW){
      I2C_log("Master responded with ACK", 3);
    } else {
      I2C_log("Master responded with N-ACK", 3);
    }

    intCfg.state = READING_PAYLOAD;
    I2C_log("State: READING_PAYLOAD", 4);
    break;

    case ACK_PAYLOAD:
    /* First SCL impulse after READING_PAYLOAD is to stop sending ACK (release SDA) and change state to LISTENING_FOR_STOP */
    releasePin(cfg->sdaOutPin);
    intCfg.DSRCounter = 0;
    intCfg.state = READING_PAYLOAD;
    I2C_log("State: READING_PAYLOAD", 4);
    break;

    default:
    break;
    }
  }

  /* START and STOP conditions (SDA change during SCL HIGH) */
  if(newSclLevel == HIGH){

    /* SDA falling edge (START conditions) */
    if(intCfg.sdaPinLevel == HIGH && newSdaLevel == LOW){
      I2C_log("Detected START", 3);
      if(intCfg.state == LISTENING_FOR_START || intCfg.state == READING_PAYLOAD){
        intCfg.DSRCounter = 0;
        intCfg.byteToSendCounter = 0;
        intCfg.dataDirection = WRITE;
        intCfg.state = START;
        I2C_log("State: START", 4);
      }
    }

    /* SDA raising edge (STOP conditions) */
    if(intCfg.sdaPinLevel == LOW && newSdaLevel == HIGH){
      I2C_log("Detected STOP", 3);
      if(intCfg.state == READING_PAYLOAD){
        if(intCfg.DSRCounter == 0){
          I2C_log("Finished transmission", 2);
        } else {
          I2C_log("Unexpected STOP", 2);
        }
      }

      intCfg.state = LISTENING_FOR_START;
      I2C_log("State: LISTENING_FOR_START", 4);
      intCfg.DSRCounter = 0;
    }
  }

  intCfg.sclPinLevel = newSclLevel;
  intCfg.sdaPinLevel = newSdaLevel;
}

uint8_t I2C_receive(bool ack){
    uint8_t receivedBits[8];
    for(uint8_t i = BYTE_SIZE - 1; i != 255; i--){
      I2C_logNum("Reading bit", i, 4);
      releasePin(cfg->sclOutPin);
      wait();
      PinLevel result = hal_pin_read(cfg->sdaInPin);
      if(result == HIGH){
        receivedBits[i] = 1;
      } else {
        receivedBits[i] = 0;
      }
      I2C_logNum("Received", receivedBits[i], 4);
      pullDownPin(cfg->sclOutPin);
      wait();
    }

    uint8_t response = binaryToDecimal(receivedBits);
    I2C_logNum("Got response: ", response, 2);
    I2C_log("Sending ACK", 3);

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

    I2C_log("Stopped sending ACK", 4);

    return response;
}

bool I2C_write(uint8_t payload){
  // TODO: check if the line is not busy
  if(cfg->role == SLAVE){
    I2C_logNum("Byte to be send", payload, 2);
    decimalToBinary(payload, intCfg.byteToSendArr);
    return true;
  }

  I2C_logNum("Sending byte", payload, 2);
  decimalToBinary(payload, intCfg.DSR);

  pullDownPin(cfg->sdaOutPin);
  for(uint8_t i = BYTE_SIZE - 1; i != 255; i--){
    I2C_logNum("Sending bit", intCfg.DSR[i], 4);
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
  bool ack = hal_pin_read(cfg->sdaInPin) == LOW ? true : false;
  pullDownPin(cfg->sclOutPin);
  // pullDownPin(cfg->sdaOutPin);
  wait();

  if(ack){
    I2C_log("Got ACK", 3);
  } else {
    I2C_log("No ACK", 3);
  }
  return ack;
}

bool I2C_writeAddress(uint8_t address, enum I2C_Data_Direction direction){

  uint8_t addr = address;
  if(direction == READ){
    I2C_logNum("Sending address with READ", address , 2);
    addr += 128;
  } else {
    I2C_logNum("Sending address with WRITE", address , 2);
  }

  return I2C_write(addr);
}

uint8_t I2C_lastByte(){
  if(cfg->role == SLAVE){
    return intCfg.receivedByte;
  }
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
  I2C_log("Sending START", 3);
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
  I2C_log("Sending RE-START", 3);
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
  I2C_log("Sending STOP", 3);
  pullDownPin(cfg->sdaOutPin);
  wait();
  releasePin(cfg->sclOutPin);
  wait();
  releasePin(cfg->sdaOutPin);
  wait();
}