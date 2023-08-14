#include "i2c.h"


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

  hal_pin_direction(cfg->SCLPin, OUTPUT);
  hal_pin_direction(cfg->SDAPin, OUTPUT);
}

void I2C_releasePin(HALPin pin){
  hal_pin_write(pin, LOW);
}

void I2C_pullDownPin(HALPin pin){
  hal_pin_write(pin, HIGH);
}

void I2C_wait() {
    _delay_ms(500);
}

/* Change of SDA HIGH -> LOW with SCL HIGH */
void I2C_sendStartCondition() {
  I2C_log("Sending START", 3);
  I2C_pullDownPin(cfg->SDAPin);
  I2C_wait();
  I2C_pullDownPin(cfg->SCLPin);
  I2C_wait();
}

/* Change of SDA LOW -> HIGH with SCL HIGH */
void I2C_sendStopCondition() {
  I2C_log("Sending STOP", 3);
  I2C_releasePin(cfg->SCLPin);
  I2C_wait();
  I2C_releasePin(cfg->SDAPin);
  I2C_wait();
}