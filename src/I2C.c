#include "i2c.h"
#define TIME_UNIT 200 // Used for duration of high and low signals
#define SHORT_TIME_UNIT 100 // Used for delay after SDA change

/*
  State transitions:
  - LISTENING_FOR_START: START
  - START: LISTEING_FOR_STOP, REPEATED_START
  - LISTEING_FOR_STOP: LISTEING_FOR_START
*/
typedef enum State {
  LISTENING_FOR_START,
  START,
  TRANSMITTING,
  RECEIVING,
  REPEATED_START,
  LISTEING_FOR_STOP
};

typedef struct {
  enum State state;
  PinLevel sclPinLevel;
  PinLevel sdaPinLevel;
  uint8_t DSR[8];        // Data Shift Register - contains bits which are currently being downloaded from the I2C bus.
  uint8_t DSRCounter;    // holds bit number in currently downloaded or uploaded byte. It can have value from 0 to 9 (this includes ACK impulse).
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

void decimalToBinary(uint8_t byte, uint8_t* arr) {
    uint8_t rem = 0;
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++) {
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
    for (uint8_t i = 0; i < 8; i++) {
        value += multiplier * arr[i];
        multiplier *= 2;
    }

    return value;
}

void I2C_read(){
  PinLevel newSclLevel = hal_pin_read(cfg->sclInPin);
  PinLevel newSdaLevel = hal_pin_read(cfg->sdaInPin);

  switch (intCfg.state){
  case LISTENING_FOR_START:
    if(newSclLevel == HIGH && (intCfg.sdaPinLevel == HIGH && newSdaLevel == LOW)){
      I2C_log("Detected START", 3);
      intCfg.state = START;
      I2C_log("State: START", 4);
    }
    break;

  case START:
    if(cfg->role == SLAVE){

      /* First SCL falling edge after START is discarded and used for state change to RECEIVING */
      if(intCfg.sclPinLevel == HIGH && newSclLevel == LOW){
        intCfg.state = RECEIVING;
        I2C_log("State: RECEIVING", 4);
      }
      
    }
    break;

  case RECEIVING:
    if(intCfg.sclPinLevel == HIGH && newSclLevel == LOW){ // detect SCL falling edge
      intCfg.DSR[intCfg.DSRCounter] = newSdaLevel == HIGH ? 1 : 0;
      I2C_logNum("Received bit: ", intCfg.DSR[intCfg.DSRCounter], 4);
      intCfg.DSRCounter++;
    }
    break;

  default:
    break;
  }

  if(newSclLevel == HIGH && (intCfg.sdaPinLevel == LOW && newSdaLevel == HIGH)){
    I2C_log("Detected STOP", 3);
    I2C_logNum("Received: ", binaryToDecimal(intCfg.DSR), 3);

    intCfg.state = LISTENING_FOR_START;
    I2C_log("State: LISTENING_FOR_START", 4);
    intCfg.DSRCounter = 0;


  }

  intCfg.sclPinLevel = newSclLevel;
  intCfg.sdaPinLevel = newSdaLevel;
}

void I2C_write(uint8_t payload){
  // TODO: check if the line is not busy
  I2C_logNum("Sending byte: ", payload, 2);
  decimalToBinary(payload, intCfg.DSR);

  I2C_sendStartCondition();
  wait();
  for(uint8_t i = 0; i < 8; i++){
    I2C_logNum("Sending bit: ", intCfg.DSR[i], 4);
    if(intCfg.DSR[i] == 0){
      releasePin(cfg->sclOutPin);
      wait();
      pullDownPin(cfg->sclOutPin);
      wait();
    } else {
      releasePin(cfg->sdaOutPin);
      _delay_ms(SHORT_TIME_UNIT);
      releasePin(cfg->sclOutPin);
      wait();
      pullDownPin(cfg->sclOutPin);
      _delay_ms(SHORT_TIME_UNIT);
      pullDownPin(cfg->sdaOutPin);
      wait();
    }
  }

  I2C_sendStopCondition();
  wait();
}

/* Change of SDA HIGH -> LOW with SCL HIGH */
void I2C_sendStartCondition() {
  I2C_log("Sending START", 3);
  pullDownPin(cfg->sdaOutPin);
  wait();
  pullDownPin(cfg->sclOutPin);
  wait();
}

/* Change of SDA LOW -> HIGH with SCL HIGH */
void I2C_sendStopCondition() {
  I2C_log("Sending STOP", 3);
  releasePin(cfg->sclOutPin);
  wait();
  releasePin(cfg->sdaOutPin);
  wait();
}