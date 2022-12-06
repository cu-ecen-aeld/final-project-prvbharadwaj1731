CC = arm-linux-gnueabihf-gcc
#TODO: Add Wall and Werror flags
CFLAGS = -static
all: build

clean: 
	rm -f *.o i2c_userspace *.elf *.map	
build:
	$(CC) $(CFLAGS) -o aesd_test i2c_userspace.c i2c_utils.h ThingSpeakLinux.c ThingsSpeakLinux.h -lm -Wall