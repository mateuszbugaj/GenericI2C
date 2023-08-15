#include "i2c.h"


/*
  State transitions:
  - LISTENING_FOR_START: START
  - START: LISTEING_FOR_STOP
  - LISTEING_FOR_STOP: LISTEING_FOR_START
*/
typedef enum State {
  LISTENING_FOR_START,
  START,
  LISTEING_FOR_STOP
};

typedef struct {
  enum State state;
  PinLevel sclPinLevel;
  PinLevel sdaPinLevel;
} I2C_Internal_Config;

/*
  DSR (Data Shift Register) 
  contains bits which are currently being downloaded from the I2C bus.
*/
uint8_t DSR[8];

/*
  DSRCounter holds bit number in currently downloaded or uploaded byte.
  It can have value from 0 to 9 (this includes ACK impulse).
*/
uint8_t DSRCounter = 0;

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
}

void releasePin(HALPin pin){
  hal_pin_write(pin, LOW);
}

void pullDownPin(HALPin pin){
  hal_pin_write(pin, HIGH);
}

void wait() {
    _delay_ms(500);
}

void I2C_read(){
  PinLevel newSclLevel = hal_pin_read(cfg->sclInPin);
  PinLevel newSdaLevel = hal_pin_read(cfg->sdaInPin);
  if(intCfg.state == LISTENING_FOR_START){
    if(newSclLevel == HIGH && (intCfg.sdaPinLevel == HIGH && newSdaLevel == LOW)){
      /* START CONDITION*/
      I2C_log("Detected START", 3);
      intCfg.state = START;
    }
  }

  if(intCfg.state == START){
    if(newSclLevel == HIGH && (intCfg.sdaPinLevel == LOW && newSdaLevel == HIGH)){
      /* START CONDITION*/
      I2C_log("Detected STOP", 3);
      intCfg.state = LISTENING_FOR_START;
    }
  }

  intCfg.sclPinLevel = newSclLevel;
  intCfg.sdaPinLevel = newSdaLevel;
}

/* Change of SDA HIGH -> LOW with SCL HIGH */
void I2C_sendStartCondition() {
  I2C_log("Sending START", 3);
  pullDownPin(cfg->sclOutPin);
  wait();
  pullDownPin(cfg->sdaOutPin);
  wait();
}

/* Change of SDA LOW -> HIGH with SCL HIGH */
void I2C_sendStopCondition() {
  I2C_log("Sending STOP", 3);
  releasePin(cfg->sdaOutPin);
  wait();
  releasePin(cfg->sclOutPin);
  wait();
}