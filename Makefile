OBJS=main.o
ELF=$(notdir $(CURDIR)).elf  
HEX=$(notdir $(CURDIR)).hex
F_CPU=16000000L

ATDIR=../Atmel.ATtiny_DFP.2.0.368


CFLAGS=-mmcu=attiny402 -B $(ATDIR)/gcc/dev/attiny402/ -O3 -Wall
CFLAGS+=-I $(ATDIR)/include/ -DF_CPU=$(F_CPU)
LDFLAGS=-mmcu=attiny402 -B$(ATDIR)/gcc/dev/attiny402/
CC=avr-gcc
LD=avr-gcc

all:    $(HEX)  

$(ELF): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)

$(HEX): $(ELF)
	avr-objcopy -O ihex -R .eeprom $< $@

flash:  $(HEX)
	pyupdi -d tiny402 -c /dev/tty.usbserial-FTF5HUAV -f $(HEX)

read-fuses:
	pyupdi -d tiny402 -c /dev/tty.usbserial-FTF5HUAV -fr

clean:
	rm -rf $(OBJS) $(ELF) $(HEX)

