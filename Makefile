CC=avr-gcc
OBJCPY=avr-objcopy
CFLAGS= -Wall -Wformat-overflow=0 -Os -std=c99 -ffunction-sections -fdata-sections -MMD -mmcu=atmega328p
LDFLAGS = -Wall -Os -Wl,--gc-sections,--relax -mmcu=atmega328p
LIBS = -lm

OBJ = twi.o gps.o pll.o usart.o morse.o pi4.o wspr.o beacon.o

%.o: %.c $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@  

beacon: $(OBJ)
	$(CC) $(LDFLAGS) -o beacon.elf $^ $(LIBS)
	$(OBJCPY) -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 beacon.elf beacon.hex
	$(OBJCPY) -O ihex -R .eeprom beacon.elf beacon.hex
	avr-size -C --mcu=atmega328p beacon.elf

burn:
	avrdude -c usbtiny -p ATMEGA328P -v -U flash:w:beacon.hex -U lfuse:w:0xe0:m -U hfuse:w:0xd8:m -U efuse:w:0xff:m

clean:
	rm -f *.o *.d *.elf *.hex
