# GenericI2C

A software-based implementation of the I2C protocol for receivers and transmitters. Uses basic HAL library implementation to manage bus on the selected platform. Doesn't use timers and ISR. </br>
Max speed of 2 bytes per second without logging.</br>
Currently works with 4 GPIO pins per microcontroler (2 for input and output).

## Example
Bi-directional communication of two microcontrollers. (Master - Slave)

![example](example.png)

Communication sequence:

```
MASTER                                  SLAVE
|                                           |
+-----------[Addres + WRITE]--------------->+
+-----------[Counter]---------------------->+
|                                           |
+-----------[Addres + READ]---------------->+
+<----------[Counter + 1]-------------------+
|                                           |
```

Code snippet:

```C++
#define MASTER_ADDR 51
#define SLAVE_ADDR 52
HAL_Pin sclOutPin, sdaOutPin, sclInPin, sdaInPin;

I2C_Config i2c_config = {
  .loggingLevel = 4,
  .sclOutPin = HAL_pinSetup(&sclOutPin, &PORTD, &PIND, 7, &DDRD),
  .sdaOutPin = HAL_pinSetup(&sdaOutPin, &PORTD, &PIND, 6, &DDRD),
  .sclInPin = HAL_pinSetup(&sclInPin, &PORTD, &PIND, 5, &DDRD),
  .sdaInPin = HAL_pinSetup(&sdaInPin, &PORTB, &PINB, 7, &DDRB),
  .timeUnit = 200,
  .print_str = &usart_print,
  .print_num = &usart_print_num,
};

HAL_Pin rolePin;
HAL_pinSetup(&rolePin, &PORTB, &PINB, 2, &DDRB);
HAL_setPinDirection(&rolePin, INPUT);

if(HAL_pinRead(&rolePin) == LOW){
  i2c_config.role = MASTER;
  i2c_config.addr = MASTER_ADDR;
  _delay_ms(500);
} else {
  i2c_config.role = SLAVE;
  i2c_config.addr = SLAVE_ADDR;
}

I2C_init(&i2c_config);

while (1) {
  if(i2c_config.role == SLAVE){
    I2C_read();
    if(I2C_newByteReceived(true)){
      uint8_t payload = I2C_lastByte();
      I2C_logNum("> Got new byte: ", payload, 1);
      I2C_write(++payload);
    }
  }

  if(i2c_config.role == MASTER){
    I2C_sendStartCondition();
    if(I2C_writeAddress(SLAVE_ADDR, WRITE)){
      I2C_write(counter);
      I2C_sendRepeatedStartCondition();
      I2C_writeAddress(SLAVE_ADDR, READ);
      uint8_t response = I2C_receive(true);
      I2C_logNum("> Got response: ", response, 1);
      counter = response;
    } else {
      I2C_logNum("> Address not responding: ", SLAVE_ADDR, 1);
      return 0;
    }
    I2C_sendStopCondition();
  }
}

```