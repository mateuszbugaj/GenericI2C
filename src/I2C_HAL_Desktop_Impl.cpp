#pragma once
#include <stdio.h>
#include <vector>

extern "C" {
#include "I2C_HAL.h"
}

HAL_LoggingCallback loggingCallback = NULL;
std::vector<HAL_Pin*> signals;

void HAL_setLoggingCallback(HAL_LoggingCallback logFunc) {
    loggingCallback = logFunc;
}

HAL_Pin* pinSetup(HAL_Pin* newPin, uint8_t* port, uint8_t pin, HAL_PullupConfig pullup){
  newPin->port = port;
  newPin->pin = pin;
  newPin->pullup = pullup;
  newPin->direction = OUTPUT;
  newPin->level = LOW;

  signals.push_back(newPin);

  return newPin;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  printf("Selecting pin %d direction: %s\n", pin->pin, direction == INPUT ? "INPUT" : "OUTPUT");
  pin->direction = direction;
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level) {
    printf("Writing to pin %d: %d\n", pin->pin, level);
    pin->level = level;
    if (loggingCallback) {
      loggingCallback(signals.data()[0], signals.size());
    }
}

HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  printf("Reading from pin %d\n", pin->pin);
  // TODO
  return pin->level;
}