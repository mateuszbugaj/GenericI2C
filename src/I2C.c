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
void defaultPrint(const char chars[], uint8_t count, ...){}
void (*printFuncPtr)(char content[], uint8_t count, ...) = &defaultPrint;
void I2C_setPrintFunc(void (*ptr)(const char[], uint8_t, ...)){
    printFuncPtr = ptr;
}

I2C_Config* cfg;

void I2C_init(I2C_Config* config){
  cfg = config;
  logging("I2C Start", 0);
}

void logging(char content[], uint8_t level) {
  if (cfg->loggingLevel >= level) {
    (*printFuncPtr)(content, 0);
    (*printFuncPtr)("\n\r", 0);
  }
}