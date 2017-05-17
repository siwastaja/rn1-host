#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>
#include <math.h>

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

int main(int argc, char** argv)
{
	if(init_uart())
	{
		fprintf(stderr, "uart initialization failed. uart is required.\n");
		return 1;
	}

	fd_set fds;
	struct timeval select_time = {0, 100};

	int fds_size = uart;
	if(STDIN_FILENO > fds_size) fds_size = STDIN_FILENO;
	fds_size+=1;

	while(1)
	{
		FD_ZERO(&fds);
		FD_SET(uart, &fds);
		FD_SET(STDIN_FILENO, &fds);
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

		lidar_scan_t* p_lid;
		if( (p_lid = get_lidar()) )
		{
			int idx_x, idx_y, offs_x, offs_y;
			printf("INFO: Got lidar scan.\n");
			if(p_lid->status & LIDAR_STATUS_SYNCED_IMAGES)
			{
				// TODO: some error checking...
				page_coords(p_lid->pos.x, p_lid->pos.y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_9pages(&world, idx_x, idx_y);

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



