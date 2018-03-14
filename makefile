MODEL=DELIVERY_BOY

CFLAGS = -D$(MODEL) -DMAP_DIR=\"/home/hrst/rn1-host\" -DSERIAL_DEV=\"/dev/serial0\" -Wall -Winline -std=c99 -g
LDFLAGS = 

DEPS = mapping.h uart.h map_memdisk.h datatypes.h hwdata.h tcp_comm.h tcp_parser.h routing.h map_opers.h pulutof.h
OBJ = rn1host.o mapping.o map_memdisk.o uart.o hwdata.o tcp_comm.o tcp_parser.o routing.o map_opers.o pulutof.o

all: rn1host

#CFLAGS += -DSIMULATE_SERIAL
CFLAGS += -DPULUTOF1
CFLAGS += -DPULUTOF_ROBOT_SER_1_TO_4
#CFLAGS += -DPULUTOF_ROBOT_SER_5_UP

%.o: %.c $(DEPS)
	gcc -c -o $@ $< $(CFLAGS) -pthread

rn1host: $(OBJ)
	gcc $(LDFLAGS) -o rn1host $^ -lm -pthread

e:
	gedit --new-window rn1host.c datatypes.h mapping.h mapping.c hwdata.h hwdata.c tcp_parser.h tcp_parser.c routing.c routing.h tof3d.h tof3d.cpp tcp_comm.c tcp_comm.h uart.c uart.h mcu_micronavi_docu.c map_memdisk.c map_memdisk.h pulutof.h pulutof.c &
