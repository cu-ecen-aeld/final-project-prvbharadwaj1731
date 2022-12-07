CC = arm-linux-gnueabihf-gcc
#TODO: Add Wall and Werror flags
CFLAGS = -static
all: build

clean: 
	rm -f *.o main *.elf *.map	
build:
	$(CC) $(CFLAGS) -o aesd_test main.c -lm -Wall