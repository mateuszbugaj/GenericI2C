#include <avr/io.h>
#include <avr/power.h>
#include <avr/delay.h>
#include "i2c.h"
#include "usart.h"

/*
+--------+
|  A168  |
|        |
|    PB0 +---> LED ---> GND 
|        |
+--------+
*/

int main(void) {
  clock_prescale_set(clock_div_1);

  HALPin led = {&PORTB, 0};
  hal_pin_direction(led, OUTPUT);

  I2C_setPrintFunc(&usart_print);
  I2C_Config i2c_config = {
    .addr = 123,
    .respondToGeneralCall = true,
    .loggingLevel = 4};

  usart_init();  
  I2C_init(&i2c_config);
  while (1) {
    hal_pin_write(led, HIGH); 
    _delay_ms(100);
    hal_pin_write(led, LOW); 
    _delay_ms(100);
  }

}