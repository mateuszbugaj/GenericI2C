#include <I2C_HAL.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

HAL_Pin* HAL_pinSetup(HAL_Pin* newPin, uint16_t* port, uint8_t pin, HAL_PullupConfig pullup){
  newPin->port = port;
  newPin->pin = pin;
  newPin->pullup = pullup;
  newPin->direction = OUTPUT;
  newPin->level = LOW;
  return newPin;
}

void HAL_setPinDirection(HAL_Pin* pin, HAL_PinDirection direction){
  pin->direction = direction;
  if(direction == INPUT){
    gpio_set_mode(*(pin->port), GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, pin->pin);
    if(pin->pullup == PULLUP_ENABLE){
      gpio_set(*(pin->port), pin->pin);
    } else {
      gpio_clear(*(pin->port), pin->pin);
    }
  } else {
    gpio_set_mode(*(pin->port), GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, pin->pin);
  }
}

void HAL_pinWrite(HAL_Pin* pin, HAL_PinLevel level){
  pin->level = level;
  if(level == HIGH){
    gpio_set(*(pin->port), pin->pin);
  } else {
    gpio_clear(*(pin->port), pin->pin);
  }
}

HAL_PinLevel HAL_pinRead(HAL_Pin* pin){
  if(gpio_get(*(pin->port), pin->pin)){
    return HIGH;
  } else {
    return LOW;
  }
}

void HAL_sleep(uint16_t ms){
  for(uint16_t i = 0; i < ms; i++){
    for(uint16_t j = 0; j < 1000; j++){
      __asm__("nop");
    }
  }
}