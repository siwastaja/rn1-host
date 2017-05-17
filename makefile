CC = gcc
LD = gcc

CFLAGS = -Wall -Winline -std=c99
LDFLAGS = 

DEPS = mapping.h uart.h map_memdisk.h datatypes.h hwdata.h
OBJ = rn1host.o mapping.o map_memdisk.o uart.o hwdata.o

all: rn1host

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

rn1host: $(OBJ)
	$(LD) $(LDFLAGS) -o rn1host $^ -lm
