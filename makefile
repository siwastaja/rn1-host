CFLAGS = -Wall -Winline -std=c99 -g
LDFLAGS = 

DEPS = mapping.h uart.h map_memdisk.h datatypes.h hwdata.h tcp_comm.h tcp_parser.h routing.h map_opers.h
OBJ = rn1host.o mapping.o map_memdisk.o uart.o hwdata.o tcp_comm.o tcp_parser.o routing.o map_opers.o

all: rn1host

tof3d.o: tof3d.cpp tof3d.h
	c++ -c -o tof3d.o -Wall -I/opt/softkinetic/DepthSenseSDK/include tof3d.cpp -pthread

%.o: %.c $(DEPS)
	gcc -c -o $@ $< $(CFLAGS) -pthread

rn1host: $(OBJ) tof3d.o
	c++ $(LDFLAGS) -o rn1host $^ -lm -L/opt/softkinetic/DepthSenseSDK/lib -lDepthSense -pthread

e:
	gedit --new-window rn1host.c datatypes.h mapping.h mapping.c hwdata.h hwdata.c tcp_parser.h tcp_parser.c routing.c routing.h tof3d.h tof3d.cpp &
