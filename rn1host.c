#define _POSIX_C_SOURCE 200809L
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
#include <time.h>
#include <pthread.h>

#include "datatypes.h"
#include "hwdata.h"
#include "map_memdisk.h"
#include "mapping.h"
#include "uart.h"
#include "tcp_comm.h"
#include "tcp_parser.h"
#include "routing.h"
#include "utlist.h"
#include "tof3d.h"

#include "mcu_micronavi_docu.c"

double subsec_timestamp()
{
	struct timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);

	return (double)spec.tv_sec + (double)spec.tv_nsec/1.0e9;
}

int mapping_on = 0;
int pos_corr_id = 42;
#define INCR_POS_CORR_ID() {pos_corr_id++; if(pos_corr_id > 99) pos_corr_id = 0;}


int map_significance_mode = MAP_SEMISIGNIFICANT_IMGS | MAP_SIGNIFICANT_IMGS;

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
lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];

int run_search()
{
	int32_t da, dx, dy;
	map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
	INCR_POS_CORR_ID();
	correct_robot_pos(da, dx, dy, pos_corr_id);

	route_unit_t *some_route = NULL;

	int ret = search_route(&world, &some_route, ANG32TORAD(cur_ang), cur_x, cur_y, dest_x, dest_y);

	route_unit_t *rt;
	int len = 0;
	DL_FOREACH(some_route, rt)
	{
//		if(rt->backmode)
//			printf(" REVERSE ");
//		else
//			printf("         ");

		int x_mm, y_mm;
		mm_from_unit_coords(rt->loc.x, rt->loc.y, &x_mm, &y_mm);					
//		printf("to %d,%d\n", x_mm, y_mm);

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
		move_to(the_route[0].x, the_route[0].y, the_route[0].backmode, (id_cnt<<4), (mapping_on==1)?30:50, 0);
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
							if(check_direct_route(cur_ang, MM_TO_UNIT(cur_x), MM_TO_UNIT(cur_y), the_route[route_pos+1].x, the_route[route_pos+1].y))
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
						move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), (mapping_on==1)?30:50, 0);
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
int charger_fwd;
int charger_first_x, charger_first_y, charger_second_x, charger_second_y;
#define CHARGER_FIRST_DIST 1000
#define CHARGER_SECOND_DIST 500
#define CHARGER_THIRD_DIST  170

/*
void conf_charger_pos_pre()  // call when the robot is *in* the charger.
{
	int32_t da, dx, dy;
	map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
	INCR_POS_CORR_ID();
	correct_robot_pos(da, dx, dy, pos_corr_id);
}
*/

void save_robot_pos()
{
	FILE* f_cha = fopen("/home/hrst/rn1-host/robot_pos.txt", "w");
	if(f_cha)
	{
		fprintf(f_cha, "%d %d %d\n", cur_ang, cur_x, cur_y);
		fclose(f_cha);
	}
}

void retrieve_robot_pos()
{
	int32_t ang;
	int x; int y;
	FILE* f_cha = fopen("/home/hrst/rn1-host/robot_pos.txt", "r");
	if(f_cha)
	{
		fscanf(f_cha, "%d %d %d", &ang, &x, &y);
		fclose(f_cha);
		set_robot_pos(ang, x, y);
	}
}

void conf_charger_pos()  // call when the robot is *in* the charger.
{
	int32_t da, dx, dy;
	map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
	INCR_POS_CORR_ID();

	int32_t cha_ang = cur_ang-da; int cha_x = cur_x+dx; int cha_y = cur_y+dy;

	correct_robot_pos(da, dx, dy, pos_corr_id);

	printf("INFO: Set charger pos at ang=%d, x=%d, y=%d\n", cha_ang, cha_x, cha_y);
	charger_first_x = (float)cha_x - cos(ANG32TORAD(cha_ang))*(float)CHARGER_FIRST_DIST;
	charger_first_y = (float)cha_y - sin(ANG32TORAD(cha_ang))*(float)CHARGER_FIRST_DIST;	
	charger_second_x = (float)cha_x - cos(ANG32TORAD(cha_ang))*(float)CHARGER_SECOND_DIST;
	charger_second_y = (float)cha_y - sin(ANG32TORAD(cha_ang))*(float)CHARGER_SECOND_DIST;
	charger_fwd = CHARGER_SECOND_DIST-CHARGER_THIRD_DIST;
	charger_ang = cha_ang;

	FILE* f_cha = fopen("/home/hrst/rn1-host/charger_pos.txt", "w");
	if(f_cha)
	{
		fprintf(f_cha, "%d %d %d %d %d %d\n", charger_first_x, charger_first_y, charger_second_x, charger_second_y, charger_ang, charger_fwd);
		fclose(f_cha);
	}
}

void read_charger_pos()
{
	FILE* f_cha = fopen("/home/hrst/rn1-host/charger_pos.txt", "r");
	if(f_cha)
	{
		fscanf(f_cha, "%d %d %d %d %d %d", &charger_first_x, &charger_first_y, &charger_second_x, &charger_second_y, &charger_ang, &charger_fwd);
		fclose(f_cha);
		printf("Info: charger position retrieved from file: %d, %d --> %d, %d, ang=%d, fwd=%d\n", charger_first_x, charger_first_y, charger_second_x, charger_second_y, charger_ang, charger_fwd);
	}
}


void request_tof_quit(void);

void* main_thread()
{
	int find_charger_state = 0;
	if(init_uart())
	{
		fprintf(stderr, "uart initialization failed.\n");
		return NULL;
	}

	if(init_tcp_comm())
	{
		fprintf(stderr, "TCP communication initialization failed.\n");
		return NULL;
	}

	srand(time(NULL));

	daiju_mode(0);
	correct_robot_pos(0,0,0, pos_corr_id);
	turn_and_go(0, 40, 30, 1);

	double chafind_timestamp = 0.0;
	int lidar_ignore_over = 0;
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

		struct timeval select_time = {0, 200};

		if(select(fds_size, &fds, NULL, NULL, &select_time) < 0)
		{
			fprintf(stderr, "select() error %d", errno);
			return NULL;
		}

		if(FD_ISSET(STDIN_FILENO, &fds))
		{
			int cmd = fgetc(stdin);
			if(cmd == 'q')
				break;
			if(cmd == 'S')
			{
				save_robot_pos();
			}
			if(cmd == 's')
			{
				retrieve_robot_pos();
			}
			if(cmd == 'c')
			{
				printf("Starting automapping from compass round.\n");
				routing_set_world(&world);
				start_automapping_from_compass();
			}
			if(cmd == 'a')
			{
				printf("Starting automapping, skipping compass round.\n");
				routing_set_world(&world);
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
//			if(cmd == 'K')
//			{
//				conf_charger_pos_pre();
//			}
			if(cmd == 'L')
			{
				conf_charger_pos();
			}
			if(cmd == 'l')
			{
				read_charger_pos();
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
			if(cmd == 'd')
			{
				dbg_test();
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
				move_to(msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode, 0, (mapping_on==1)?30:50, 1);
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
			else if(ret == TCP_CR_CHARGE_MID)
			{
				read_charger_pos();
				find_charger_state = 1;
			}
			else if(ret == TCP_CR_MODE_MID)
			{
				printf("INFO: Request for MODE %d\n", msg_cr_mode.mode);
				switch(msg_cr_mode.mode)
				{
					case 0:
					{
						motors_on = 1;
						daiju_mode(0);
						stop_automapping();
						mapping_on = 0;
					} break;

					case 1:
					{
						motors_on = 1;
						daiju_mode(0);
						stop_automapping();
						mapping_on = 1;
					} break;

					case 2:
					{
						motors_on = 1;
						daiju_mode(0);
						routing_set_world(&world);
						start_automapping_skip_compass();
						mapping_on = 1;
					} break;

					case 3:
					{
						motors_on = 1;
						daiju_mode(0);
						routing_set_world(&world);
						start_automapping_from_compass();
						mapping_on = 1;
					} break;

					case 4:
					{
						stop_automapping();
						do_follow_route = 0;
						motors_on = 1;
						daiju_mode(1);
						mapping_on = 0;
					} break;

					case 5:
					{
						stop_automapping();
						do_follow_route = 0;
						motors_on = 0;
						mapping_on = 1;
					} break;

					case 6:
					{
						stop_automapping();
						do_follow_route = 0;
						motors_on = 0;
						mapping_on = 0;
					} break;

					case 7:
					{
						conf_charger_pos();
					} break;

					default: break;
				}
			}
			else if(ret == TCP_CR_MANU_MID)
			{
				#define MANU_FWD   10
				#define MANU_BACK  11
				#define MANU_LEFT  12
				#define MANU_RIGHT 13
				stop_automapping();
				daiju_mode(0);
				motors_on = 1;
				printf("INFO: Manual OP %d\n", msg_cr_manu.op);
				switch(msg_cr_manu.op)
				{
					case MANU_FWD:
						turn_and_go(cur_ang, 100, 10, 1);
					break;
					case MANU_BACK:
						turn_and_go(cur_ang, -100, 10, 1);
					break;
					case MANU_LEFT:
						turn_and_go(cur_ang-10*ANG_1_DEG, 0, 10, 1);
					break;
					case MANU_RIGHT:
						turn_and_go(cur_ang+10*ANG_1_DEG, 0, 10, 1);
					break;
					default:
					break;
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
				int stop_reason = cur_xymove.feedback_stop_flags;
				printf("INFO: Feedback module reported: %s\n", MCU_FEEDBACK_COLLISION_NAMES[stop_reason]);
				if(mapping_on) map_collision_obstacle(&world, cur_ang, cur_x, cur_y, stop_reason, cur_xymove.stop_xcel_vector_valid,
					cur_xymove.stop_xcel_vector_ang_rad);
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
				if(sq(cur_x-charger_first_x) + sq(cur_y-charger_first_y) > sq(300))
				{
					printf("INFO: We are not at the first charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					printf("INFO: At first charger point, turning for charger.\n");
					turn_and_go(charger_ang, 0, 23, 1);
					find_charger_state++;
					chafind_timestamp = subsec_timestamp();

				}
			}
		}
		else if(find_charger_state == 3)
		{
			double stamp;
			if( (stamp=subsec_timestamp()) > chafind_timestamp+3.0)
			{
				chafind_timestamp = stamp;

				printf("INFO: Turned at first charger point, mapping lidars for exact pos.\n");

				int32_t da, dx, dy;
				map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
				INCR_POS_CORR_ID();
				correct_robot_pos(da, dx, dy, pos_corr_id);
				lidar_ignore_over = 0;
				find_charger_state++;
			}
		}
		else if(find_charger_state == 4)
		{
			if(lidar_ignore_over && subsec_timestamp() > chafind_timestamp+4.0)
			{
				printf("INFO: Going to second charger point.\n");
				move_to(charger_second_x, charger_second_y, 0, 0x7f, 20, 1);
				find_charger_state++;
			}
		}
		else if(find_charger_state == 5)
		{
			if(cur_xymove.id == 0x7f && cur_xymove.remaining < 10)
			{
				if(sq(cur_x-charger_second_x) + sq(cur_y-charger_second_y) > sq(180))
				{
					printf("INFO: We are not at the second charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					turn_and_go(charger_ang, charger_fwd, 20, 1);
					chafind_timestamp = subsec_timestamp();
					find_charger_state++;
				}
			}
		}
		else if(find_charger_state == 6)
		{
			double stamp;
			if( (stamp=subsec_timestamp()) > chafind_timestamp+3.0)
			{
				chafind_timestamp = stamp;
				turn_and_go(charger_ang, 0, 23, 1);
				find_charger_state++;
			}
		}
		else if(find_charger_state == 7)
		{
			if(subsec_timestamp() > chafind_timestamp+1.5)
			{
				printf("INFO: Requesting charger mount.\n");
				hw_find_charger();
				find_charger_state = 0;
			}
		}

		route_fsm();
		autofsm();

		lidar_scan_t* p_lid;

		if( (p_lid = get_significant_lidar()) || (p_lid = get_basic_lidar()) )
		{
			static int hwdbg_cnt = 0;
			hwdbg_cnt++;
			if(hwdbg_cnt > 4)
			{
				if(tcp_client_sock >= 0) tcp_send_hwdbg(hwdbg);
				hwdbg_cnt = 0;
			}

			static int lidar_send_cnt = 0;
			lidar_send_cnt++;
			if(lidar_send_cnt > 8)
			{
				if(tcp_client_sock >= 0) tcp_send_lidar(p_lid);
				lidar_send_cnt = 0;
			}

			static int lidar_ignore_cnt = 0;

			if(p_lid->id != pos_corr_id)
			{

				lidar_ignore_cnt++;

//				if(p_lid->significant_for_mapping) 
//				printf("INFO: Ignoring lidar scan with id=%d (significance=%d).\n", p_lid->id, p_lid->significant_for_mapping);

				if(lidar_ignore_cnt > 50)
				{
					lidar_ignore_cnt = 0;
					printf("WARN: lidar id was stuck, fixing...\n");
					INCR_POS_CORR_ID();
					correct_robot_pos(0, 0, 0, pos_corr_id);

				}
			}
			else
			{
				lidar_ignore_cnt = 0;
				lidar_ignore_over = 1;

				int idx_x, idx_y, offs_x, offs_y;
	//			printf("INFO: Got lidar scan.\n");

				cur_ang = p_lid->robot_pos.ang; cur_x = p_lid->robot_pos.x; cur_y = p_lid->robot_pos.y;
//				printf("INFO: cur_ang = %6.1fdeg\n", ANG32TOFDEG(cur_ang));

				static int curpos_send_cnt = 0;
				curpos_send_cnt++;
				if(curpos_send_cnt > 2)
				{
					if(tcp_client_sock >= 0)
					{
						msg_rc_pos.ang = cur_ang>>16;
						msg_rc_pos.x = cur_x;
						msg_rc_pos.y = cur_y;
						tcp_send_msg(&msgmeta_rc_pos, &msg_rc_pos);

						tcp_send_hmap(TOF3D_HMAP_XSPOTS, TOF3D_HMAP_YSPOTS, cur_ang, cur_x, cur_y, TOF3D_HMAP_SPOT_SIZE, tof3d_objmap);

					}
					curpos_send_cnt = 0;
				}

				page_coords(p_lid->robot_pos.x, p_lid->robot_pos.y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_9pages(&world, idx_x, idx_y);

				if(mapping_on)
				{
					// Clear any walls and items within the robot:
					clear_within_robot(&world, p_lid->robot_pos);
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
//					lidar_send_cnt = 0;
//					if(tcp_client_sock >= 0) tcp_send_lidar(p_lid);

					static int n_lidars_to_map = 0;
					static lidar_scan_t* lidars_to_map[20];
					if(mapping_on)
					{
						if(p_lid->is_invalid)
						{
							if(n_lidars_to_map < 8)
							{
								printf("INFO: Got DISTORTED significant lidar scan, have too few lidars -> mapping queue reset\n");
								n_lidars_to_map = 0;
							}
							else
							{
								printf("INFO: Got DISTORTED significant lidar scan, running mapping early on previous images\n");
								int32_t da, dx, dy;
								map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
								INCR_POS_CORR_ID();
								correct_robot_pos(da, dx, dy, pos_corr_id);
								n_lidars_to_map = 0;
							}
						}
						else
						{
//							printf("INFO: Got significant(%d) lidar scan, adding to the mapping queue(%d).\n", p_lid->significant_for_mapping, n_lidars_to_map);
							lidars_to_map[n_lidars_to_map] = p_lid;

							n_lidars_to_map++;
							if((good_time_for_lidar_mapping && n_lidars_to_map > 10) || n_lidars_to_map > 19)
							{
								if(good_time_for_lidar_mapping) good_time_for_lidar_mapping = 0;
								int32_t da, dx, dy;
								map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
								INCR_POS_CORR_ID();
								correct_robot_pos(da, dx, dy, pos_corr_id);
								n_lidars_to_map = 0;
							}
						}
					}
					else
					{
						n_lidars_to_map = 0;
					}

				}

			}

			if(motors_on)
				send_keepalive();
			else
				release_motors();
		}

		sonar_scan_t* p_son;
		if( (p_son = get_sonar()) )
		{
			static int sonar_send_cnt = 0;
			sonar_send_cnt++;
			if(sonar_send_cnt > 8)
			{
				if(tcp_client_sock >= 0) tcp_send_sonar(p_son);
				sonar_send_cnt = 0;
			}
			if(mapping_on)
				map_sonar(&world, p_son);
		}

		static double prev_sync = 0;
		double stamp;
		if( (stamp=subsec_timestamp()) > prev_sync+5.0)
		{
			prev_sync = stamp;

			int idx_x, idx_y, offs_x, offs_y;
			page_coords(cur_x, cur_y, &idx_x, &idx_y, &offs_x, &offs_y);

			// Do some "garbage collection" by disk-syncing and deallocating far-away map pages.
			unload_map_pages(&world, idx_x, idx_y);

			// Sync all changed map pages to disk
			if(save_map_pages(&world))
			{
				if(tcp_client_sock >= 0) tcp_send_sync_request();
			}
			if(tcp_client_sock >= 0) tcp_send_battery();

			static int automap_started = 0;
			if(!automap_started)
			{
				automap_started = 1;
//				start_automapping_from_compass();
			}

			fflush(stdout); // syncs log file.

		}
	}


	request_tof_quit();

	return NULL;
}



void* start_tof(void*);

int main(int argc, char** argv)
{
	pthread_t thread_main, thread_tof;

	uint8_t calib_tof = 0;
	if(argc == 2 && argv[1][0] == 'c')
		calib_tof = 1;

	int ret;

	if(!calib_tof)
	{
		if( (ret = pthread_create(&thread_main, NULL, main_thread, NULL)) )
		{
			printf("ERROR: main thread creation, ret = %d\n", ret);
			return -1;
		}
	}
	if( (ret = pthread_create(&thread_tof, NULL, start_tof, (void*)&calib_tof)) )
	{
		printf("ERROR: tof3d thread creation, ret = %d\n", ret);
		return -1;
	}

	if(!calib_tof)
		pthread_join(thread_main, NULL);

	pthread_join(thread_tof, NULL);

	return 0;
}
