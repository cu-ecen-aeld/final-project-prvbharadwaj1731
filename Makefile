CC = arm-linux-gnueabihf-gcc
#TODO: Add Wall and Werror flags
CFLAGS = -g
all: build

clean: 
	rm -f *.o i2c_userspace *.elf *.map	
build:
	$(CC) $(CFLAGS)  -c -o i2c_userspace.o i2c_userspace.c
	$(CC) $(CFLAGS) -I/ i2c_userspace.o -o i2c_userspace