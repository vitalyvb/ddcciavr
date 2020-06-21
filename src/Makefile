#PREFIX=/usr/local/avr/bin/
CC=$(PREFIX)avr-gcc
SIZE=$(PREFIX)avr-size
OBJCOPY=$(PREFIX)avr-objcopy
OBJDUMP=$(PREFIX)avr-objdump
CFLAGS=-pipe -mmcu=atmega328 -Os -g -Wall
CFLAGS+=-fpack-struct -fshort-enums -funsigned-bitfields -funsigned-char
LDFLAGS=-g -Wl,-Map,ddcci.map -Wl,-m,avr4 -Wl,-N -mmcu=atmega328
#LDFLAGS+=-Wl,-u,vfprintf -lprintf_min

all: ddcci.elf lst text

ddcci.hex: ddcci.elf
	$(OBJCOPY) -O ihex $< $@

ddcci.elf: ddcci.o twi_slave.o twi_proto.o vcp.o buttons.o xlibc.o nvram.o
	$(CC) $(LDFLAGS) -o $@ $^ -lc
	$(SIZE) $^ $@

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f ddcci.hex ddcci.elf *.o

lst:  ddcci.lst
text: ddcci.hex ddcci.bin ddcci.srec

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@
%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@
%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

program: ddcci.hex
	avrdude -b 57600 -P /dev/ttyUSB0 -c arduino -p m328p -v -U flash:w:ddcci.hex -B 4
