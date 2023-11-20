#include <I2C_HAL.h>
#include <I2C_HAL_STM32.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint32_t portRegister, uint16_t pinNumber){
  newPin->portRegister = portRegister;
  newPin->pinNumber = pinNumber;
  return newPin;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  pin->direction = direction;
  if(direction == INPUT){
    gpio_set_mode(pin->portRegister, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, pin->pinNumber);
  } else {
    gpio_set_mode(pin->portRegister, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pin->pinNumber);
  }
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level){
  pin->level = level;
  if(level == HIGH){
    gpio_set(pin->portRegister, pin->pinNumber);
  } else {
    gpio_clear(pin->portRegister, pin->pinNumber);
  }
}

HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  if(gpio_get(pin->portRegister, pin->pinNumber)){
    return HIGH;
  } else {
    return LOW;
  }
}

void HAL_sleep(uint16_t ms){
  for(uint16_t i = 0; i < ms; i++){
    for(uint16_t j = 0; j < 10000; j++){
      __asm__("nop");
    }
  }
}