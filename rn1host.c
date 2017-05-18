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
#include "tcp_comm.h"
#include "tcp_parser.h"

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

uint32_t robot_id = 0xacdcabba; // Hopefully unique identifier for the robot.

extern world_t world;
#define BUFLEN 2048

void send_keepalive()
{
	uint8_t buf[3] = {0x8f, 0x00, 0xff};
	if(write(uart, buf, 3) != 3)
	{
		printf("uart write error\n");
	}
}



int main(int argc, char** argv)
{
	if(init_uart())
	{
		fprintf(stderr, "uart initialization failed.\n");
		return 1;
	}

	if(init_tcp_comm())
	{
		fprintf(stderr, "TCP communication initialization failed.\n");
		return 1;
	}

	while(1)
	{
		// Calculate fd_set size (biggest fd+1)
		int fds_size = uart;
		if(tcp_listener_sock > fds_size) fds_size = tcp_listener_sock;
		if(tcp_client_sock > fds_size) fds_size = tcp_client_sock;
		if(STDIN_FILENO > fds_size) fds_size = STDIN_FILENO;
		fds_size+=1;


		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(uart, &fds);
		FD_SET(STDIN_FILENO, &fds);
		FD_SET(tcp_listener_sock, &fds);
		if(tcp_client_sock >= 0)
			FD_SET(tcp_client_sock, &fds);

		struct timeval select_time = {0, 100};

		if(select(fds_size, &fds, NULL, NULL, &select_time) < 0)
		{
			fprintf(stderr, "select() error %d", errno);
			return 1;
		}

		if(FD_ISSET(STDIN_FILENO, &fds))
		{
			int cmd = fgetc(stdin);
			if(cmd == 'q')
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

		if(tcp_client_sock >= 0 && FD_ISSET(tcp_client_sock, &fds))
		{
			int ret = handle_tcp_client();
			printf("handle_tcp_client() returned %d\n", ret);

			if(ret == TCP_CR_DEST_MID)
			{
				printf("  ---> DEST params: %d, %d\n", msg_cr_dest.x, msg_cr_dest.y);
			}
		}

		if(FD_ISSET(tcp_listener_sock, &fds))
		{
			handle_tcp_listener();
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

			send_keepalive();			
		}

//		int idx_x, idx_y, offs_x, offs_y;
//		page_coords(x, y, &idx_x, &idx_y, &offs_x, &offs_y);

		// Do some "garbage collection":
//		unload_map_pages(&world,);
	}

	return 0;
}



