#include "i2c.h"
#define TIME_UNIT 200 // Used for duration of high and low signals
#define SHORT_TIME_UNIT 50 // Used for visual distinction (at least 1ms)
#define BYTE_SIZE 8

typedef enum State {
  LISTENING_FOR_START,
  START,
  RE_START,
  READING_ADDRESS,
  ACK_ADDRESS,
  LISTENING_FOR_STOP,
  ADDRESSED,
  READING_PAYLOAD,
  ACK_PAYLOAD
};

typedef struct {
  enum State state;
  PinLevel sclPinLevel;
  PinLevel sdaPinLevel;
  uint8_t DSR[8];        // Data Shift Register - contains bits which are currently being downloaded from the I2C bus.
  uint8_t DSRCounter;    // Holds bit number in currently downloaded or uploaded byte. It can have value from 0 to 9 (this includes ACK impulse).
  uint8_t receivedByte;  // Holds last whole byte decoted to decimal from DSR.
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

  I2C_logNum("Addr", cfg->addr, 1);

  hal_pin_direction(cfg->sclOutPin, OUTPUT);
  hal_pin_direction(cfg->sdaOutPin, OUTPUT);
  hal_pin_direction(cfg->sclInPin, INPUT);
  hal_pin_direction(cfg->sdaInPin, INPUT);

  intCfg.state = LISTENING_FOR_START;
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
  _delay_ms(TIME_UNIT);
}

void waitShort(){
  _delay_ms(SHORT_TIME_UNIT);
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
|.       +START                  .              -(release SDA)       .                 .               +LISTENING_FOR_STOP     |
|+LISTENING_FOR_START            +ACK_ADDRESS/LISTENING_FOR_STOP     +READING_PAYLOAD  +ACK_PAYLOAD    -(release SDA)          |
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
    intCfg.DSR[intCfg.DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum("Received bit: ", intCfg.DSR[intCfg.DSRCounter], 4);
    intCfg.DSRCounter++;

    if(intCfg.DSRCounter == BYTE_SIZE){
      intCfg.receivedByte = binaryToDecimal(intCfg.DSR);
      intCfg.DSRCounter = 0;
      I2C_logNum("Received address: ", intCfg.receivedByte, 3);
      if(intCfg.receivedByte == cfg->addr){
        I2C_logNum("Address match: ", cfg->addr, 2);
        pullDownPin(cfg->sdaOutPin);
        intCfg.state = ACK_ADDRESS;
        I2C_log("State: ACK_ADDRESS", 4);
      } else {
        intCfg.state = LISTENING_FOR_STOP;
        I2C_log("State: LISTENING_FOR_STOP", 4);
      }
    }
    break;

    case ACK_ADDRESS:
    /* First SCL impulse after READING_ADDRESS is to stop sending ACK (release SDA) and change state to RE_START */
    releasePin(cfg->sdaOutPin);
    break;

    case RE_START:
    /* First SCL impulse after RE-START is used for state change to READING_PAYLOAD */
    intCfg.state = READING_PAYLOAD;
    I2C_log("State: READING_PAYLOAD", 4);
    break;

    case READING_PAYLOAD:
    intCfg.DSR[intCfg.DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
    I2C_logNum("Received bit: ", intCfg.DSR[intCfg.DSRCounter], 4);
    intCfg.DSRCounter++;

    if(intCfg.DSRCounter == BYTE_SIZE){
      intCfg.receivedByte = binaryToDecimal(intCfg.DSR);
      I2C_logNum("Received payload: ", intCfg.receivedByte, 2);
      pullDownPin(cfg->sdaOutPin);
      intCfg.state = ACK_PAYLOAD;
      I2C_log("State: ACK_PAYLOAD", 4);
    }
    break;

    case ACK_PAYLOAD:
    /* First SCL impulse after READING_PAYLOAD is to stop sending ACK (release SDA) and change state to LISTENING_FOR_STOP */
    releasePin(cfg->sdaOutPin);
    intCfg.state = LISTENING_FOR_STOP;
    I2C_log("State: LISTENING_FOR_STOP", 4);
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
      if(intCfg.state == LISTENING_FOR_START){
        intCfg.state = START;
        I2C_log("State: START", 4);
      }

      if(intCfg.state == ACK_ADDRESS){
        intCfg.state = RE_START;
        I2C_log("State: RE_START", 4);
      }
    }

    /* SDA raising edge (STOP conditions) */
    if(intCfg.sdaPinLevel == LOW && newSdaLevel == HIGH){
      I2C_log("Detected STOP", 3);
      if(intCfg.state == READING_PAYLOAD){
        I2C_log("Unexpected STOP", 2);
      }

      if(intCfg.state == LISTENING_FOR_STOP){
        I2C_log("Finished transmission", 2);
      }

      intCfg.state = LISTENING_FOR_START;
      I2C_log("State: LISTENING_FOR_START", 4);
      intCfg.DSRCounter = 0;
    }
  }

  intCfg.sclPinLevel = newSclLevel;
  intCfg.sdaPinLevel = newSdaLevel;
}

bool I2C_write(uint8_t payload){
  // TODO: check if the line is not busy
  I2C_logNum("Sending byte: ", payload, 2);
  decimalToBinary(payload, intCfg.DSR);

  I2C_sendStartCondition();
  wait();
  for(uint8_t i = 0; i < BYTE_SIZE; i++){
    I2C_logNum("Sending bit: ", intCfg.DSR[i], 4);
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
  wait();

  if(ack){
    I2C_log("Got ACK", 3);
  } else {
    I2C_log("No ACK", 3);
  }
  return ack;
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
  wait();
  pullDownPin(cfg->sclOutPin);
  wait();
  releasePin(cfg->sdaOutPin);
  wait();
  releasePin(cfg->sclOutPin);
  wait();
  pullDownPin(cfg->sdaOutPin);
  wait();
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