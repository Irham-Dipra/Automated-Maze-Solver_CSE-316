MCU=atmega32
F_CPU=1000000UL
CC=avr-gcc
OBJCOPY=avr-objcopy
CFLAGS=-Wall -Os -DF_CPU=${F_CPU} -mmcu=${MCU} -x c
TARGET=atmega32Test
SRCS=atmega32Test.cpp

all: ${TARGET}.hex

${TARGET}.hex: ${TARGET}.elf
	${OBJCOPY} -O ihex -R .eeprom $< $@

${TARGET}.elf: ${SRCS}
	${CC} ${CFLAGS} -o $@ $<

flash: ${TARGET}.hex
	avrdude -c usbasp -p m32 -B 4 -U flash:w:$<:i

clean:
	rm -f *.o *.elf *.hex
