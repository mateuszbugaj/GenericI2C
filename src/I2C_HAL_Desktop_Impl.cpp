#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

extern "C" {
#include "I2C_HAL.h"
#include "I2C_HAL_DESKTOP.h"
}

volatile uint8_t pinCounter = 0;
HAL_Pin* scl = HAL_pinSetup(SCL);
HAL_Pin* sda = HAL_pinSetup(SDA);
std::vector<HAL_Pin*> registeredPins;

HAL_Pin* HAL_pinSetup(HAL_PinRole pinRole){
  std::string roleString;
  switch (pinRole){
    case SDA:
      roleString = "SDA";
      break;
    case SCL:
      roleString = "SCL";
      break;
    case SDA_IN:
      roleString = "SDA_IN";
      break;
    case SDA_OUT:
      roleString = "SDA_OUT";
      break;
    case SCL_IN:
      roleString = "SCL_IN";
      break;
    case SCL_OUT:
      roleString = "SCL_OUT";
      break;

    default:
      roleString = "UNKNOWN";
      break;
  };

  printf("Creating pin %d: %s\n", pinCounter, roleString.c_str());
  HAL_Pin* pin = new HAL_Pin;
  pin->pinNumber = pinCounter++;
  pin->pinRole = pinRole;
  
  if(pinRole == SCL || pinRole == SDA){
    pin->level = HIGH;
    pin->direction = INPUT_OUTPUT;
    std::cout << "> " << pin->pinRole << '\n';
    std::cout << "> " << pin->level << '\n';
  } else if(pinRole == SCL_IN || pinRole == SDA_IN){
    pin->direction = INPUT;
  } else if(pinRole == SCL_OUT || pinRole == SDA_OUT){
    pin->direction = OUTPUT;
  }

  registeredPins.push_back(pin);
  return pin;
}

HAL_Pin* HAL_SclPin(){
  return scl;
}

HAL_Pin* HAL_SdaPin(){
  return sda;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  pin->direction = direction;
}

void updateBusLevels(){
  HAL_PinLevel sclLevel = HIGH;
  HAL_PinLevel sdaLevel = HIGH;
  for(HAL_Pin* registeredPin : registeredPins){
    if(registeredPin->pinRole == SCL_OUT && registeredPin->level == HIGH) sclLevel = LOW;
    if(registeredPin->pinRole == SDA_OUT && registeredPin->level == HIGH) sdaLevel = LOW;
  }

  scl->level = sclLevel;
  sda->level = sdaLevel;
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level) {
  if(pin->direction == INPUT){
    std::cout << "WARNING: Writing to an input pin. Returning.\n";
    return;
  }

  pin->level = level;

  updateBusLevels();
}

HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  if(pin->pinRole == SCL_IN || pin->pinRole == SCL) return scl->level;
  if(pin->pinRole == SDA_IN || pin->pinRole == SDA) return sda->level;

  return pin->level;
}

void HAL_sleep(uint16_t ms){
  std::chrono::milliseconds sleepTimespan(ms);
  std::this_thread::sleep_for(sleepTimespan);
}