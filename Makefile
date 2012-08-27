
all:
	avr-gcc -mmcu=atmega168p -Os -mcall-prologues -Wall -std=c99 \
    -pedantic -Wstrict-prototypes -Wundef \
    -funsigned-bitfields -ffunction-sections -fpack-struct \
    -ffreestanding -o bt.obj *.c
	avr-objcopy -O ihex -R .eeprom bt.obj bt.hex

upload:
	avrdude -c stk500v2 -p m168 -P usb -U'flash:w:bt.hex:i'

clean:
	rm bt.hex bt.obj
