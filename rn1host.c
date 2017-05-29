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

int32_t cur_ang;
int32_t cur_x;
int32_t cur_y;

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

	int cnt = 0;
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
			if(ret == TCP_CR_DEST_MID)
			{
				printf("  ---> DEST params: %d, %d\n", msg_cr_dest.x, msg_cr_dest.y);
				move_to(msg_cr_dest.x, msg_cr_dest.y);
			}
		}

		if(FD_ISSET(tcp_listener_sock, &fds))
		{
			handle_tcp_listener();
		}


		lidar_scan_t* p_lid;

		if( (p_lid = get_significant_lidar()) || (p_lid = get_basic_lidar()) )
		{
			if(tcp_client_sock >= 0) tcp_send_hwdbg(hwdbg);

			static int lidar_send_cnt = 0;
			lidar_send_cnt++;
			if(lidar_send_cnt > 5)
			{
				if(tcp_client_sock >= 0) tcp_send_lidar(p_lid);
				lidar_send_cnt = 0;
			}

			int idx_x, idx_y, offs_x, offs_y;
//			printf("INFO: Got lidar scan.\n");

			cur_ang = p_lid->robot_pos.ang; cur_x = p_lid->robot_pos.x; cur_y = p_lid->robot_pos.y;
			if(tcp_client_sock >= 0)
			{
				msg_rc_pos.ang = cur_ang>>16;
				msg_rc_pos.x = cur_x;
				msg_rc_pos.y = cur_y;
				tcp_send_msg(&msgmeta_rc_pos, &msg_rc_pos);
			}

			page_coords(p_lid->robot_pos.x, p_lid->robot_pos.y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);
			if(p_lid->significant_for_mapping)
			{
				lidar_send_cnt = 0;
				if(tcp_client_sock >= 0) tcp_send_lidar(p_lid);
				printf("INFO: Got significant lidar scan, adding to map.\n");
				static int n_lidars_to_map = 0;
				static lidar_scan_t* lidars_to_map[10];
				lidars_to_map[n_lidars_to_map] = p_lid;

				n_lidars_to_map++;
				if(n_lidars_to_map == 10)
				{
					int32_t da, dx, dy;
					map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
					correct_robot_pos(da, dx, dy);
					n_lidars_to_map = 0;
				}

			}

			send_keepalive();			
		}

		sonar_scan_t* p_son;
		if( (p_son = get_sonar()) )
		{
			if(tcp_client_sock >= 0) tcp_send_sonar(p_son);

			int idx_x, idx_y, offs_x, offs_y;

			for(int i=0; i<3; i++)
			{
				if(!p_son->scan[i].valid) continue;

				page_coords(p_son->scan[i].x, p_son->scan[i].y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_9pages(&world, idx_x, idx_y);
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].result = world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_ITEM;
				//world.changed[idx_x][idx_y] = 1;
			}
		}


		cnt++;
		if(cnt > 100000)
		{
			cnt = 0;

			int idx_x, idx_y, offs_x, offs_y;
			page_coords(cur_x, cur_y, &idx_x, &idx_y, &offs_x, &offs_y);

			// Do some "garbage collection" by disk-syncing and deallocating far-away map pages.
			unload_map_pages(&world, idx_x, idx_y);

			// Sync all changed map pages to disk
			save_map_pages(&world);
		}

	}

	return 0;
}



