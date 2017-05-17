#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>

#include "datatypes.h"
#include "hwdata.h"
#include "map_memdisk.h"
#include "mapping.h"
#include "uart.h"

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

uint32_t robot_id = 0xacdcabba; // Hopefully unique identifier for the robot.

extern world_t world;
#define BUFLEN 2048
int main(int argc, char** argv)
{
	int port = 22334;
	uint8_t buf[BUFLEN];

	if(init_uart())
	{
		fprintf(stderr, "uart initialization failed. uart is required.\n");
		return 1;
	}

	fd_set fds;
	struct timeval select_time = {0, 100};


	/* Create the socket and set it up to accept connections. */
	struct sockaddr_in si_me, si_other, si_subscriber;
	 
	int udpsock, recv_len;
	unsigned int slen = sizeof(si_other);
	 
	//create a UDP socket
	if ((udpsock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
		printf("Error opening UDP socket.\n");
		return 1;
	}
	 
	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));
	 
	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(port);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);
	 
	//bind socket to port
	if( bind(udpsock , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
	{
		printf("Error binding the port to the socket.\n");
		return 1;
	}



	int fds_size = uart;
	if(udpsock > fds_size) fds_size = udpsock;
	if(STDIN_FILENO > fds_size) fds_size = STDIN_FILENO;
	fds_size+=1;

	while(1)
	{
		FD_ZERO(&fds);
		FD_SET(uart, &fds);
		FD_SET(STDIN_FILENO, &fds);
		FD_SET(udpsock, &fds);

		if(select(fds_size, &fds, NULL, NULL, &select_time) < 0)
		{
			fprintf(stderr, "select() error %d", errno);
			return 1;
		}

		if(FD_ISSET(STDIN_FILENO, &fds))
		{
			int cmd = fgetc(stdin);
			if(cmd == 'Q')
				break;
			if(cmd == 's')
			{
				save_map_pages(&world);
			}
		}

		if(FD_ISSET(uart, &fds))
		{
			handle_uart();
		}

		if(FD_ISSET(udpsock, &fds))
		{
			if ((recv_len = recvfrom(udpsock, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen)) == -1)
			{
				printf("recvfrom() failed");
				return 1;
			}
			else
			{
				printf("Relaying message to robot.\n");
				if(write(uart, &buf[1], recv_len-1) != recv_len-1)
				{
					printf("uart write error\n");
				}
			}

		}

		lidar_scan_t* p_lid;
		if( (p_lid = get_lidar()) )
		{
			int idx_x, idx_y, offs_x, offs_y;
			printf("INFO: Got lidar scan.\n");
			page_coords(p_lid->pos.x, p_lid->pos.y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);
			if(p_lid->status & LIDAR_STATUS_SYNCED_IMAGES)
			{
				// TODO: some error checking...
				printf("INFO: Lidar scan images are synced.\n");
				for(int i = 0; i < 360; i++)
				{
					int len = p_lid->scan[i];
					if(len > 0)
					{
						double co = cos(2.0*M_PI*(   (((double)p_lid->pos.ang)/4294967296.0)  +  ((double)i/360.0)   ));
						double si = sin(2.0*M_PI*(   (((double)p_lid->pos.ang)/4294967296.0)  +  ((double)i/360.0)   ));

						int x, y;
						for(int d = 0; d < len; d+=40)
						{
							x = p_lid->pos.x + co * d;
							y = p_lid->pos.y + si * d;

							page_coords(x, y, &idx_x, &idx_y, &offs_x, &offs_y);
							world.pages[idx_x][idx_y]->units[offs_x][offs_y].result = world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest = UNIT_FREE;
						}
						x = p_lid->pos.x + co * len;
						y = p_lid->pos.y + si * len;

						page_coords(x, y, &idx_x, &idx_y, &offs_x, &offs_y);
						world.pages[idx_x][idx_y]->units[offs_x][offs_y].result = world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest = UNIT_WALL;
					}
				}
			}
			
		}

//		int idx_x, idx_y, offs_x, offs_y;
//		page_coords(x, y, &idx_x, &idx_y, &offs_x, &offs_y);

		// Do some "garbage collection":
//		unload_map_pages(&world,);
	}

	return 0;
}



