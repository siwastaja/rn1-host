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
#include "routing.h"
#include "utlist.h"

#include "mcu_micronavi_docu.c"

#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

int mapping_on = 0;

int map_significance_mode = MAP_SIGNIFICANT_IMGS;

int motors_on = 1;

uint32_t robot_id = 0xacdcabba; // Hopefully unique identifier for the robot.

extern world_t world;
#define BUFLEN 2048

int32_t cur_ang;
int32_t cur_compass_ang;
int compass_round_active;
int32_t cur_x, cur_y;
int32_t dest_x, dest_y;

typedef struct
{
	int x;
	int y;
	int backmode;
	int take_next_early;
	int timeout;
} route_point_t;

#define THE_ROUTE_MAX 200
route_point_t the_route[THE_ROUTE_MAX];

int do_follow_route = 0;
int route_pos = 0;
int start_route = 0;
int id_cnt = 0;
int good_time_for_lidar_mapping = 0;

#define sq(x) ((x)*(x))

#define NUM_LATEST_LIDARS_FOR_ROUTING_START 7
static lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];

int run_search()
{
	int32_t da, dx, dy;
	map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
	correct_robot_pos(da, dx, dy);


	route_unit_t *some_route = NULL;

	int ret = search_route(&world, &some_route, ANG32TORAD(cur_ang), cur_x, cur_y, dest_x, dest_y);

	route_unit_t *rt;
	int len = 0;
	DL_FOREACH(some_route, rt)
	{
		if(rt->backmode)
			printf(" REVERSE ");
		else
			printf("         ");

		int x_mm, y_mm;
		mm_from_unit_coords(rt->loc.x, rt->loc.y, &x_mm, &y_mm);					
		printf("to %d,%d\n", x_mm, y_mm);

		the_route[len].x = x_mm; the_route[len].y = y_mm; the_route[len].backmode = rt->backmode;
		the_route[len].take_next_early = 100;
		len++;
		if(len >= THE_ROUTE_MAX)
			break;
	}

	for(int i = 0; i < len; i++)
	{
		if(i < len-1)
		{
			float dist = sqrt(sq(the_route[i].x-the_route[i+1].x) + sq(the_route[i].y-the_route[i+1].y));
			int new_early = dist/10;
			if(new_early < 50) new_early = 50;
			else if(new_early > 250) new_early = 250;
			the_route[i].take_next_early = new_early;
		}
	}

	the_route[len-1].take_next_early = 20;


	tcp_send_route(&some_route);

	if(some_route)
	{
		do_follow_route = len;
		start_route = 1;
		route_pos = 0;
		id_cnt++; if(id_cnt > 7) id_cnt = 0;
	}
	else do_follow_route = 0;

	return ret;

}

void route_fsm()
{
	static int stop_flag_cnt = 0;
	static int first_fail = 0;

	if(start_route && route_pos == 0)
	{
		printf("Start going id=%d!\n", id_cnt<<4);
		move_to(the_route[0].x, the_route[0].y, the_route[0].backmode, (id_cnt<<4), (mapping_on==1)?30:50);
		start_route = 0;
		first_fail = 1;
	}
	if(do_follow_route)
	{
		int id = cur_xymove.id;

		if(((id&0b1110000) == (id_cnt<<4)) && ((id&0b1111) == ((route_pos)&0b1111)))
		{
			if(cur_xymove.micronavi_stop_flags || cur_xymove.feedback_stop_flags)
			{
				stop_flag_cnt++;

				if(stop_flag_cnt > first_fail?50000:150000) // todo: proper timing.
				{
					first_fail = 0;
					stop_flag_cnt = 0;
					printf("Robot stopped, retrying routing.\n");
					daiju_mode(0);
					if(run_search() == 1)
					{
						printf("Routing failed in start, going to daiju mode.\n");
						daiju_mode(1);
					}
				}
			}
			else
			{
				stop_flag_cnt = 0;

				if(cur_xymove.remaining < 250)
				{
					good_time_for_lidar_mapping = 1;
				}

				if(cur_xymove.remaining < the_route[route_pos].take_next_early)
				{
					if(route_pos < do_follow_route-1)
					{
						route_pos++;

						// Check if we can skip some points:
						while(the_route[route_pos].backmode == 0 && route_pos < do_follow_route-1)
						{
							if(check_direct_route(cur_ang, cur_x, cur_y, the_route[route_pos+1].x, the_route[route_pos+1].y))
							{
								printf("INFO: skipping point (%d, %d), going directly to (%d, %d)\n", the_route[route_pos].x,
								       the_route[route_pos].y, the_route[route_pos+1].x, the_route[route_pos+1].y);
								route_pos++;
							}
							else
							{
								break;
							}
						}
						printf("Take the next, id=%d!\n", (id_cnt<<4) | ((route_pos)&0b1111));
						move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), (mapping_on==1)?30:50);
						first_fail = 1;
					}
					else
					{
						printf("Done following the route.\n");
						do_follow_route = 0;
					}
				}
			}
		}

	}

}

int32_t charger_ang;
int charger_first_x, charger_first_y, charger_second_x, charger_second_y;
#define CHARGER_FIRST_DIST 600
#define CHARGER_SECOND_DIST 250

void conf_charger_pos(int32_t cha_ang, int cha_x, int cha_y)  // Coordinates when the robot is *in* the charger.
{
	int32_t da, dx, dy;
	map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
	correct_robot_pos(da, dx, dy);

	printf("INFO: Set charger pos at ang=%d, x=%d, y=%d\n", cha_ang, cha_x, cha_y);
	charger_first_x = (float)cha_x - cos(ANG32TORAD(cha_ang))*(float)CHARGER_FIRST_DIST;
	charger_first_y = (float)cha_y - sin(ANG32TORAD(cha_ang))*(float)CHARGER_FIRST_DIST;	
	charger_second_x = (float)cha_x - cos(ANG32TORAD(cha_ang))*(float)CHARGER_SECOND_DIST;
	charger_second_y = (float)cha_y - sin(ANG32TORAD(cha_ang))*(float)CHARGER_SECOND_DIST;
}


int main(int argc, char** argv)
{
	int find_charger_state = 0;
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

	daiju_mode(0);

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
			if(cmd == 'c')
			{
				printf("Starting automapping from compass round.\n");
				start_automapping_from_compass();
			}
			if(cmd == 'a')
			{
				printf("Starting automapping, skipping compass round.\n");
				start_automapping_skip_compass();
			}
			if(cmd == 'w')
			{
				printf("Stopping automapping.\n");
				stop_automapping();
			}
			if(cmd == '0')
			{
				set_robot_pos(0,0,0);
			}
			if(cmd == 'm')
			{
				if(mapping_on)
				{
					mapping_on = 0;
					printf("Turned mapping off.\n");
				}
				else
				{
					mapping_on = 1;
					printf("Turned mapping on.\n");
				}
			}
			if(cmd == 'M')
			{
				mapping_on = 2;
				printf("Turned mapping to fast mode.\n");
			}
			if(cmd == 'L')
			{
				conf_charger_pos(cur_ang, cur_x, cur_y);
			}
			if(cmd == 'l')
			{
				find_charger_state = 1;
			}
			if(cmd == 'v')
			{
				if(motors_on)
				{
					motors_on = 0;
					printf("Robot is free to move manually.\n");
				}
				else
				{
					motors_on = 1;
					printf("Robot motors enabled again.\n");
				}
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
				motors_on = 1;
				daiju_mode(0);

				printf("  ---> DEST params: X=%d Y=%d backmode=%d\n", msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode);
				move_to(msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode, 0, (mapping_on==1)?30:50);
				do_follow_route = 0;
			}
			else if(ret == TCP_CR_ROUTE_MID)
			{
				printf("  ---> ROUTE params: X=%d Y=%d dummy=%d\n", msg_cr_route.x, msg_cr_route.y, msg_cr_route.dummy);

				dest_x = msg_cr_route.x; dest_y = msg_cr_route.y;

				motors_on = 1;
				daiju_mode(0);

				if(run_search() == 1)
				{
					printf("INFO: Routing fails in the start, daijuing for a while to get a better position.\n");
					daiju_mode(1);
					printf("INFO: (Please retry after some time.)\n");
				}

			}
		}

		if(FD_ISSET(tcp_listener_sock, &fds))
		{
			handle_tcp_listener();
		}

//		static int prev_compass_ang = 0;

//		if(cur_compass_ang != prev_compass_ang)
//		{
//			prev_compass_ang = cur_compass_ang;
//			printf("INFO: Compass ang=%.1f deg\n", ANG32TOFDEG(cur_compass_ang));
//		}

		static int micronavi_stop_flags_printed = 0;

		if(cur_xymove.micronavi_stop_flags)
		{
			if(!micronavi_stop_flags_printed)
			{
				micronavi_stop_flags_printed = 1;
				printf("INFO: MCU-level micronavigation reported a stop. Stop reason flags:\n");
				for(int i=0; i<32; i++)
				{
					if(cur_xymove.micronavi_stop_flags&(1UL<<i))
					{
						printf("bit %2d: %s\n", i, MCU_NAVI_STOP_NAMES[i]);
					}
				}

				printf("Actions taken during stop condition:\n");
				for(int i=0; i<32; i++)
				{
					if(cur_xymove.micronavi_action_flags&(1UL<<i))
					{
						printf("bit %2d: %s\n", i, MCU_NAVI_ACTION_NAMES[i]);
					}
				}

				printf("\n");
			}
		}
		else
			micronavi_stop_flags_printed = 0;

		static int feedback_stop_flags_processed = 0;

		if(cur_xymove.feedback_stop_flags)
		{
			if(!feedback_stop_flags_processed)
			{
				feedback_stop_flags_processed = 1;
				int stop_reason = cur_xymove.feedback_stop_flags&0b11;
				printf("INFO: Collision reported: %s\n", MCU_FEEDBACK_COLLISION_NAMES[stop_reason]);
				if(mapping_on) map_collision_obstacle(&world, cur_ang, cur_x, cur_y, stop_reason);
			}
		}
		else
			feedback_stop_flags_processed = 0;

		if(find_charger_state == 1)
		{
			dest_x = charger_first_x; dest_y = charger_first_y;

			printf("Searching dest_x=%d  dest_y=%d\n", dest_x, dest_y);

			motors_on = 1;
			daiju_mode(0);
			if(run_search() != 0)
			{
				printf("Finding charger (first point) failed.\n");
				find_charger_state = 0;
			}
			else
				find_charger_state++;
		}
		else if(find_charger_state == 2)
		{
			if(!do_follow_route)
			{
				if(sq(cur_x-charger_first_x) + sq(cur_y-charger_first_y) > sq(200))
				{
					printf("We are not at the first charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					move_to(charger_second_x, charger_second_y, 0, 0x7f, 25);
					find_charger_state++;
				}
			}
		}
		else if(find_charger_state == 3)
		{
			if(cur_xymove.id == 0x7f && cur_xymove.remaining < 20)
			{
				if(sq(cur_x-charger_second_x) + sq(cur_y-charger_second_y) > sq(140))
				{
					printf("We are not at the second charger point, giving up.\n");
				}
				else				
					hw_find_charger();

				find_charger_state = 0;
			}
		}

		route_fsm();
		autofsm();

		lidar_scan_t* p_lid;

		static int ignore_lidars = 0;
		if( (p_lid = get_significant_lidar()) || (p_lid = get_basic_lidar()) )
		{
			if(ignore_lidars)
			{
				if(p_lid->significant_for_mapping) printf("INFO: Ignoring significant lidar scan.\n");
			}
			else
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

				if(mapping_on)
				{
					// Clear any walls and items within the robot:

					for(int xx = -120; xx <= 120; xx++)
					{
						for(int yy = -120; yy <= 120; yy++)
						{
							page_coords(p_lid->robot_pos.x + xx, p_lid->robot_pos.y + yy, &idx_x, &idx_y, &offs_x, &offs_y);
							world.pages[idx_x][idx_y]->units[offs_x][offs_y].result = UNIT_MAPPED;
							MINUS_SAT_0(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
						}
					}

				}


				// Keep a pointer list of a few latest lidars; significant or insignificant will do.
				// This list is used to do last-second mapping before routing, to get good starting position.
				for(int i = NUM_LATEST_LIDARS_FOR_ROUTING_START-1; i >= 1; i--)
				{
					lidars_to_map_at_routing_start[i] = lidars_to_map_at_routing_start[i-1];
				}
				lidars_to_map_at_routing_start[0] = p_lid;
				if(p_lid->significant_for_mapping & map_significance_mode)
				{
					lidar_send_cnt = 0;
					if(tcp_client_sock >= 0) tcp_send_lidar(p_lid);

					if(mapping_on)
					{
						static int n_lidars_to_map = 0;
						static lidar_scan_t* lidars_to_map[16];
						if(p_lid->is_invalid)
						{
							if(n_lidars_to_map < 5)
							{
								printf("INFO: Got DISTORTED significant lidar scan, have too few lidars -> mapping queue reset\n");
								map_next_with_larger_search_area();
								n_lidars_to_map = 0;
							}
							else
							{
								printf("INFO: Got DISTORTED significant lidar scan, running mapping early on previous images\n");
								int32_t da, dx, dy;
								map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
								correct_robot_pos(da, dx, dy);
								// Get and ignore all lidar images:
								ignore_lidars = 50000;
								n_lidars_to_map = 0;
								map_next_with_larger_search_area();

							}
						}
						else
						{
							printf("INFO: Got significant(%d) lidar scan, adding to the mapping queue.\n", p_lid->significant_for_mapping);
							lidars_to_map[n_lidars_to_map] = p_lid;

							n_lidars_to_map++;
							if((good_time_for_lidar_mapping && n_lidars_to_map > 7) || n_lidars_to_map > 15)
							{
								if(good_time_for_lidar_mapping) good_time_for_lidar_mapping = 0;
								int32_t da, dx, dy;
								map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
								correct_robot_pos(da, dx, dy);
								// Get and ignore all lidar images:
								ignore_lidars = 50000;
								n_lidars_to_map = 0;
							}
						}
					}

				}

			}

			if(motors_on)
				send_keepalive();
			else
				release_motors();
		}
		else
		{
			if(ignore_lidars>0) ignore_lidars--; // stop ignoring lidars once no more is coming.
		}

		sonar_scan_t* p_son;
		if( (p_son = get_sonar()) )
		{
			if(tcp_client_sock >= 0) tcp_send_sonar(p_son);
			if(mapping_on)
				map_sonar(&world, p_son);
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
			if(tcp_client_sock >= 0) tcp_send_battery();
		}

	}

	return 0;
}



