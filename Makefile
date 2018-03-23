#
# $Id$
#

#.DEFAULT:
#.SUFFIXES: .c .o .hex .elf
#.PHONY:
.SECONDARY:

all: main.hex

CFLAGS+= -I. -Os -DF_CPU=16000000UL -mmcu=atmega328p --std=c99
CFLAGS+= -Wall
#CFLAGS+= -save-temps
#CFLAGS+= -fno-unwind-tables -fno-asynchronous-unwind-tables
CFLAGS+= -ffunction-sections -fdata-sections
#CFLAGS+= -MD -MP -MT $(*F).o -MF $(*F).d
CFLAGS+= -ffunction-sections -fdata-sections 

LDFLAGS+= -s -DF_CPU=16000000UL -mmcu=atmega328p
LDFLAGS+= -Wl,--gc-sections
#LDFLAGS+= -lm
#LDFLAGS+= -Wl,-u,vfprintf -lprintf_flt

MAIN_OBJS+= main.o fifo.o tools.o shell.o
MAIN_OBJS+= uart.o
MAIN_OBJS+= mcp.o

main.elf: $(MAIN_OBJS)
	avr-gcc $(LDFLAGS) -o $@ $(^F)
	avr-size --format=berkeley $@

%.o: %.c
	avr-gcc $(CFLAGS) -c -o $@ $<

%.elf: %.o
	avr-gcc $(LDFLAGS) -o $@ $<

%.hex: %.elf
	avr-objcopy -O ihex -R .eeprom $< $@


%.upl: %.hex
	avrdude -qq -c arduino -p ATMEGA328P -P /dev/ttyU0 -b 115200 -U flash:w:$<

upload: main.upl

backup:
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyU0 -b 57600 -U flash:r:backup.hex:i

clean:
	rm -f *.i *.o *.s *.elf *~ *.hex *.d

#EOF
