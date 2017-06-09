CC = gcc
LD = gcc

CFLAGS = -Wall -Winline -std=c99
LDFLAGS = 

DEPS = mapping.h uart.h map_memdisk.h datatypes.h hwdata.h tcp_comm.h tcp_parser.h routing.h
OBJ = rn1host.o mapping.o map_memdisk.o uart.o hwdata.o tcp_comm.o tcp_parser.o routing.o

all: rn1host

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

rn1host: $(OBJ)
	$(LD) $(LDFLAGS) -o rn1host $^ -lm

e:
	gedit --new-window rn1host.c datatypes.h mapping.h mapping.c uart.h uart.c hwdata.c hwdata.c tcp_parser.h tcp_parser.c routing.c routing.h &
