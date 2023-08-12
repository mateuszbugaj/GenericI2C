CC := avr-gcc
CFLAGS := -Os -DF_CPU=8000000UL -mmcu=atmega168

I2C_LIBRARY_DIR := ./
I2C_LIBRARY_NAME := I2C
I2C_LIBRARY_OBJ_DIR := $(I2C_LIBRARY_DIR)src/
I2C_LIBRARY_INC_DIR := $(I2C_LIBRARY_DIR)inc/
I2C_LIBRARY_SRC := $(wildcard $(I2C_LIBRARY_OBJ_DIR)*.c)
I2C_LIBRARY_OBJ := $(I2C_LIBRARY_SRC:.c=.o)
I2C_LIBRARY_AR := ar -rcs
HAL_INC_DIR = lib/HALibrary/inc

# default rule to build .a library
all: $(I2C_LIBRARY_NAME).a

$(I2C_LIBRARY_NAME).a: $(I2C_LIBRARY_OBJ)
	$(I2C_LIBRARY_AR) $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -I$(I2C_LIBRARY_INC_DIR) -I$(HAL_INC_DIR) -c $< -o $@

clean:
	rm -f $(I2C_LIBRARY_NAME).a $(I2C_LIBRARY_OBJ)