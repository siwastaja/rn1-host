/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



*/

//#define PULUTOF1_GIVE_RAWS

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

#include "pulutof.h"

#include "mcu_micronavi_docu.c"

#define DEFAULT_SPEEDLIM 45
#define MAX_CONFIGURABLE_SPEEDLIM 70

int verbose_mode = 0;

int max_speedlim = DEFAULT_SPEEDLIM;
int cur_speedlim = DEFAULT_SPEEDLIM;

#define SPEED(x_) do{ cur_speedlim = ((x_)>max_speedlim)?(max_speedlim):(x_); } while(0);

double subsec_timestamp()
{
	struct timespec spec;
	clock_gettime(CLOCK_MONOTONIC, &spec);

	return (double)spec.tv_sec + (double)spec.tv_nsec/1.0e9;
}

int mapping_on = 1;
int live_obstacle_checking_on = 1;
int pos_corr_id = 42;
#define INCR_POS_CORR_ID() {pos_corr_id++; if(pos_corr_id > 99) pos_corr_id = 0;}


int map_significance_mode = MAP_SEMISIGNIFICANT_IMGS | MAP_SIGNIFICANT_IMGS;

int motors_on = 1;

uint32_t robot_id = 0xacdcabba; // Hopefully unique identifier for the robot.

extern world_t world;
#define BUFLEN 2048

int32_t cur_ang, cur_x, cur_y;
double robot_pos_timestamp;
int32_t cur_compass_ang;
int compass_round_active;

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
int the_route_len = 0;

int do_follow_route = 0;
int route_finished_or_notfound = 0;
int lookaround_creep_reroute = 0;
int route_pos = 0;
int start_route = 0;
int id_cnt = 1;
int good_time_for_lidar_mapping = 0;

#define sq(x) ((x)*(x))

#define NUM_LATEST_LIDARS_FOR_ROUTING_START 7
lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];

void send_info(info_state_t state)
{
	if(tcp_client_sock >= 0) tcp_send_info_state(state);
}

int32_t prev_search_dest_x, prev_search_dest_y;
int run_search(int32_t dest_x, int32_t dest_y, int dont_map_lidars, int no_tight)
{
	send_info(INFO_STATE_THINK);

	prev_search_dest_x = dest_x;
	prev_search_dest_y = dest_y;

	if(!dont_map_lidars)
	{
		int32_t da, dx, dy;
		map_lidars(&world, NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start, &da, &dx, &dy);
		INCR_POS_CORR_ID();
		correct_robot_pos(da, dx, dy, pos_corr_id);
	}

	route_unit_t *some_route = NULL;

	int ret = search_route(&world, &some_route, ANG32TORAD(cur_ang), cur_x, cur_y, dest_x, dest_y, no_tight);

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


	tcp_send_route(cur_x, cur_y, &some_route);

	if(some_route)
	{
		the_route_len = len;
		do_follow_route = 1;
		start_route = 1;
		route_pos = 0;
		route_finished_or_notfound = 0;
		id_cnt++; if(id_cnt > 7) id_cnt = 1;
	}
	else
	{
		do_follow_route = 0;
		route_finished_or_notfound = 1;
		send_info(INFO_STATE_IDLE);
	}

	lookaround_creep_reroute = 0;

	return ret;

}

int rerun_search()
{
	return run_search(prev_search_dest_x, prev_search_dest_y, 0, 1);
}

static int maneuver_cnt = 0; // to prevent too many successive maneuver operations
void do_live_obstacle_checking()
{
	if(the_route[route_pos].backmode == 0)
	{
		int32_t target_x, target_y;
		int dx = the_route[route_pos].x - cur_x;
		int dy = the_route[route_pos].y - cur_y;

		int32_t dist_to_next = sqrt(sq(dx)+sq(dy));

		// Obstacle avoidance looks towards the next waypoint; if more than max_dist_to_next away,
		// it looks towards the staight line from cur_pos to the next waypoint, but only for the lenght of max_dist_to_next.
		// If there are obstacles on the straight line-of-sight, extra waypoint is searched for so that the number of
		// obstacles are minimized.
		// so target_x, target_y is only the target which is looked at. It's never a move_to target directly, the next waypoint is.

		const int32_t max_dist_to_next = 1200;
		if(dist_to_next < max_dist_to_next)
		{
			target_x = the_route[route_pos].x;
			target_y = the_route[route_pos].y;
		}
		else
		{
			float ang_to_target = atan2(dy, dx);
			target_x = cur_x + max_dist_to_next*cos(ang_to_target);
			target_y = cur_y + max_dist_to_next*sin(ang_to_target);
		}

		int hitcnt = check_direct_route_non_turning_hitcnt_mm(cur_x, cur_y, target_x, target_y);

#if 0
		if(hitcnt > 0 && maneuver_cnt < 2)
		{
			// See what happens if we steer left or right

			int best_hitcnt = 9999;
			int best_drift_idx = 0;
			int best_angle_idx = 0;
			int best_new_x = 0, best_new_y = 0;

			const int side_drifts[12] = {320,-320, 240,-240,200,-200,160,-160,120,-120,80,-80};
			const float drift_angles[4] = {M_PI/6.0, M_PI/8.0, M_PI/12.0, M_PI/16.0};

			int predicted_cur_x = cur_x + cos(ANG32TORAD(cur_ang))*(float)cur_speedlim*2.0;
			int predicted_cur_y = cur_y + sin(ANG32TORAD(cur_ang))*(float)cur_speedlim*2.0;

			for(int angle_idx=0; angle_idx<4; angle_idx++)
			{
				for(int drift_idx=0; drift_idx<12; drift_idx++)
				{
					int new_x, new_y;
					if(side_drifts[drift_idx] > 0)
					{
						new_x = predicted_cur_x + cos(ANG32TORAD(cur_ang)+drift_angles[angle_idx])*side_drifts[drift_idx];
						new_y = predicted_cur_y + sin(ANG32TORAD(cur_ang)+drift_angles[angle_idx])*side_drifts[drift_idx];
					}
					else
					{
						new_x = predicted_cur_x + cos(ANG32TORAD(cur_ang)-drift_angles[angle_idx])*(-1*side_drifts[drift_idx]);
						new_y = predicted_cur_y + sin(ANG32TORAD(cur_ang)-drift_angles[angle_idx])*(-1*side_drifts[drift_idx]);
					}
					int drifted_hitcnt = check_direct_route_hitcnt_mm(cur_ang, new_x, new_y, the_route[route_pos].x, the_route[route_pos].y);
//					printf("a=%.1f deg  drift=%d mm  cur(%d,%d) to(%d,%d)  hitcnt=%d\n",
//						RADTODEG(drift_angles[angle_idx]), side_drifts[drift_idx], predicted_cur_x, predicted_cur_y, new_x, new_y, drifted_hitcnt);
					if(drifted_hitcnt <= best_hitcnt)
					{
						best_hitcnt = drifted_hitcnt;
						best_drift_idx = drift_idx;
						best_angle_idx = angle_idx;
						best_new_x = new_x; best_new_y = new_y;
					}
				}
			}

			if(best_hitcnt < hitcnt && best_hitcnt < 2)
			{

//				do_follow_route = 0;
//				lookaround_creep_reroute = 0;
//				stop_movement();

				if( (abs(side_drifts[best_drift_idx]) < 50) || ( abs(side_drifts[best_drift_idx]) < 100 && drift_angles[best_angle_idx] < M_PI/13.0))
				{
					SPEED(18);
					limit_speed(cur_speedlim);
//					printf("!!!!!!!!!!   Steering is almost needed (not performed) to maintain line-of-sight, hitcnt now = %d, optimum drift = %.1f degs, %d mm (hitcnt=%d), cur(%d,%d) to(%d,%d)\n",
//						hitcnt, RADTODEG(drift_angles[best_angle_idx]), side_drifts[best_drift_idx], best_hitcnt, cur_x, cur_y, best_new_x, best_new_y);
//					if(tcp_client_sock > 0) tcp_send_dbgpoint(cur_x, cur_y, 210, 210,   110, 1);
//					if(tcp_client_sock > 0) tcp_send_dbgpoint(best_new_x, best_new_y,   0, 255,   110, 1);
				}
				else
				{
					printf("Steering is needed, hitcnt now = %d, optimum drift = %.1f degs, %d mm (hitcnt=%d), cur(%d,%d) to(%d,%d)\n", 
						hitcnt, RADTODEG(drift_angles[best_angle_idx]), side_drifts[best_drift_idx], best_hitcnt, cur_x, cur_y, best_new_x, best_new_y);
					if(tcp_client_sock > 0) tcp_send_dbgpoint(cur_x, cur_y, 200, 200,   0, 1);
					if(tcp_client_sock > 0) tcp_send_dbgpoint(best_new_x, best_new_y,   0, 40,   0, 1);
					if(tcp_client_sock > 0) tcp_send_dbgpoint(target_x, target_y, 0, 130, 230, 1);

					// Do the steer
					id_cnt = 0; // id0 is reserved for special maneuvers during route following.
					move_to(best_new_x, best_new_y, 0, (id_cnt<<4) | ((route_pos)&0b1111), 12, 2 /* auto backmode*/);
					send_info((side_drifts[best_drift_idx] > 0)?INFO_STATE_RIGHT:INFO_STATE_LEFT);
					maneuver_cnt++;
				}
			}
			else
			{
//				printf("!!!!!!!!  Steering cannot help in improving line-of-sight.\n");
#endif
				if(hitcnt < 3)
				{
//					printf("!!!!!!!!!!!  Direct line-of-sight to the next point has 1..2 obstacles, slowing down.\n");
					SPEED(18);
					limit_speed(cur_speedlim);
				}
				else
				{
//					printf("Direct line-of-sight to the next point has disappeared! Trying to solve.\n");
					SPEED(18);
					limit_speed(cur_speedlim);
					stop_movement();
					lookaround_creep_reroute = 1;
				}
#if 0
			}
		}
#endif
	}
}

void route_fsm()
{
	static int micronavi_stops = 0;
	static double timestamp;
	static int creep_cnt;

	if(lookaround_creep_reroute)
	{
		if(check_direct_route_non_turning_mm(cur_x, cur_y, the_route[route_pos].x, the_route[route_pos].y))
		{
			printf("Direct line-of-sight has appeared to the next waypoint, resuming following the route.\n");
			lookaround_creep_reroute = 0;
			do_follow_route = 1;
			id_cnt++; if(id_cnt > 7) id_cnt = 1;
			send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);
			move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), cur_speedlim, 0);
		}
	}

	const float lookaround_turn = 10.0;

	if(lookaround_creep_reroute == 1)
	{
		do_follow_route = 0;
		start_route = 0;

		printf("Lookaround, creep & reroute procedure started; backing off 50 mm.\n");
		turn_and_go_abs_rel(cur_ang, -50, 13, 1);
		timestamp = subsec_timestamp();
		lookaround_creep_reroute++;
	}
	else if(lookaround_creep_reroute == 2)
	{
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+1.0)
		{
			if(doing_autonomous_things())
			{
				printf("Robot is mapping autonomously: no need to clear the exact route right now, skipping lookaround & creep\n");
				rerun_search();
				lookaround_creep_reroute = 0;
			}
			else
			{
				int dx = the_route[route_pos].x - cur_x;
				int dy = the_route[route_pos].y - cur_y;
				float ang = atan2(dy, dx) /*<- ang to dest*/ - DEGTORAD(lookaround_turn);

				if(test_robot_turn_mm(cur_x, cur_y, ANG32TORAD(cur_ang),  ang))
				{
					//printf("Can turn to %.1f deg, doing it.\n", -1*lookaround_turn);
					turn_and_go_abs_rel(RADTOANG32(ang), 0, 13, 1);
				}
				else
				{
					//printf("Can't turn to %.1f deg, wiggling a bit.\n", -1*lookaround_turn);
					turn_and_go_abs_rel(cur_ang-4*ANG_1_DEG, 0, 13, 1);
				}
				timestamp = subsec_timestamp();
				lookaround_creep_reroute++;
			}
		}
	}
	else if(lookaround_creep_reroute == 3)
	{
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+1.0)
		{
			int dx = the_route[route_pos].x - cur_x;
			int dy = the_route[route_pos].y - cur_y;
			float ang = atan2(dy, dx) /*<- ang to dest*/ - DEGTORAD(1.8*lookaround_turn);

			if(test_robot_turn_mm(cur_x, cur_y, ANG32TORAD(cur_ang),  ang))
			{
				//printf("Can turn to %.1f deg, doing it.\n", -1.8*lookaround_turn);
				turn_and_go_abs_rel(RADTOANG32(ang), -20, 13, 1);
			}
			else
			{
				//printf("Can't turn to %.1f deg, wiggling a bit.\n", -1.8*lookaround_turn);
				turn_and_go_abs_rel(cur_ang-4*ANG_1_DEG, 0, 13, 1);
			}
			timestamp = subsec_timestamp();
			lookaround_creep_reroute++;
		}
	}
	else if(lookaround_creep_reroute == 4)
	{
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+1.0)
		{
			int dx = the_route[route_pos].x - cur_x;
			int dy = the_route[route_pos].y - cur_y;
			float ang = atan2(dy, dx) /*<- ang to dest*/ + DEGTORAD(lookaround_turn);

			if(test_robot_turn_mm(cur_x, cur_y, ANG32TORAD(cur_ang),  ang))
			{
				//printf("Can turn to %.1f deg, doing it.\n",lookaround_turn);
				turn_and_go_abs_rel(RADTOANG32(ang), 0, 13, 1);
			}
			else
			{
				//printf("Can't turn to %.1f deg, wiggling a bit.\n",lookaround_turn);
				turn_and_go_abs_rel(cur_ang+12*ANG_1_DEG, 0, 13, 1);
			}
			timestamp = subsec_timestamp();
			lookaround_creep_reroute++;
		}
	}
	else if(lookaround_creep_reroute == 5)
	{
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+1.0)
		{
			int dx = the_route[route_pos].x - cur_x;
			int dy = the_route[route_pos].y - cur_y;
			float ang = atan2(dy, dx) /*<- ang to dest*/ + DEGTORAD(1.8*lookaround_turn);

			if(test_robot_turn_mm(cur_x, cur_y, ANG32TORAD(cur_ang),  ang))
			{
				//printf("Can turn to %.1f deg, doing it.\n", 1.8*lookaround_turn);
				turn_and_go_abs_rel(RADTOANG32(ang), 0, 13, 1);
			}
			else
			{
				//printf("Can't turn to %.1f deg, wiggling a bit.\n", 1.8*lookaround_turn);
				turn_and_go_abs_rel(cur_ang+4*ANG_1_DEG, 0, 13, 1);
			}
			timestamp = subsec_timestamp();
			lookaround_creep_reroute++;
		}
	}
	else if(lookaround_creep_reroute == 6)
	{
		creep_cnt = 0;
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+1.0)
		{
			int dx = the_route[route_pos].x - cur_x;
			int dy = the_route[route_pos].y - cur_y;
			float ang = atan2(dy, dx) /*<- ang to dest*/;

			if(test_robot_turn_mm(cur_x, cur_y, ANG32TORAD(cur_ang),  ang))
			{
				//printf("Can turn towards the dest, doing it.\n");
				turn_and_go_abs_rel(RADTOANG32(ang), 50, 13, 1);
			}
			else
			{
				printf("Can't turn towards the dest, rerouting.\n");
				if(rerun_search() == 1)
				{
					printf("Routing failed in start, going to daiju mode for a while.\n");
					send_info(INFO_STATE_DAIJUING);
					daiju_mode(1);
					lookaround_creep_reroute = 8;
				}
				else
				{
					printf("Routing succeeded, or failed later. Stopping lookaround, creep & reroute procedure.\n");
					lookaround_creep_reroute = 0;
				}
			}
			timestamp = subsec_timestamp();
			lookaround_creep_reroute++;
		}
	}
	else if(lookaround_creep_reroute == 7)
	{
		static double time_interval = 2.5;
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+time_interval)
		{
			int dx = the_route[route_pos].x - cur_x;
			int dy = the_route[route_pos].y - cur_y;
			int dist = sqrt(sq(dx)+sq(dy));
			if(dist > 300 && creep_cnt < 3)
			{
				float ang = atan2(dy, dx) /*<- ang to dest*/;
				int creep_amount = 100;
				int dest_x = cur_x + cos(ang)*creep_amount;
				int dest_y = cur_y + sin(ang)*creep_amount;
				int hitcnt = check_direct_route_non_turning_hitcnt_mm(cur_x, cur_y, dest_x, dest_y);
				if(hitcnt < 1)
				{
					//printf("Can creep %d mm towards the next waypoint, doing it\n", creep_amount);
					time_interval = 2.5;
					turn_and_go_abs_rel(RADTOANG32(ang) + ((creep_cnt&1)?(5*ANG_1_DEG):(-5*ANG_1_DEG)), creep_amount, 15, 1);
				}
				else
				{
					creep_cnt=99;
				}
				creep_cnt++;
			}
			else
			{
				printf("We have creeped enough (dist to waypoint=%d, creep_cnt=%d), no line of sight to the waypoint, trying to reroute\n",
					dist, creep_cnt);
				if(rerun_search() == 1)
				{
					printf("Routing failed in start, going to daiju mode for a while.\n");
					daiju_mode(1);
					send_info(INFO_STATE_DAIJUING);
					lookaround_creep_reroute++;
				}
				else
				{
					printf("Routing succeeded, or failed later. Stopping lookaround, creep & reroute procedure.\n");
					lookaround_creep_reroute = 0;
				}

			}
			timestamp = subsec_timestamp();
		}
	}
	else if(lookaround_creep_reroute >= 8 && lookaround_creep_reroute < 12)
	{
		double stamp;
		if( (stamp=subsec_timestamp()) > timestamp+5.0)
		{
			printf("Daijued enough.\n");
			daiju_mode(0);
			if(rerun_search() == 1)
			{
				printf("Routing failed in start, going to daiju mode for a bit more...\n");
				daiju_mode(1);
					send_info(INFO_STATE_DAIJUING);
				lookaround_creep_reroute++;
				timestamp = subsec_timestamp();
			}
			else
			{
				printf("Routing succeeded, or failed later. Stopping lookaround, creep & reroute procedure.\n");
				lookaround_creep_reroute = 0;
			}

		}
	}
	else if(lookaround_creep_reroute == 12)
	{
		printf("Giving up lookaround, creep & reroute procedure!\n");
		lookaround_creep_reroute = 0;
	}

	if(start_route)
	{
		printf("Start going id=%d!\n", id_cnt<<4);
		move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4), cur_speedlim, 0);
		send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);

		start_route = 0;
	}

	if(do_follow_route)
	{
		int id = cur_xymove.id;

		if(((id&0b1110000) == (id_cnt<<4)) && ((id&0b1111) == ((route_pos)&0b1111)))
		{
			if(cur_xymove.micronavi_stop_flags || cur_xymove.feedback_stop_flags)
			{
				if(micronavi_stops < 7)
				{
					printf("Micronavi STOP, entering lookaround_creep_reroute\n");
					micronavi_stops++;
					lookaround_creep_reroute = 1;
				}
				else
				{
					printf("Micronavi STOP, too many of them already, rerouting.\n");
					if(rerun_search() == 1)
					{
						printf("Routing failed in start, todo: handle this situation.\n");
					}
					else
					{
						printf("Routing succeeded, or failed later.\n");
					}
				}
			}
			else if(id_cnt == 0) // Zero id move is a special move during route following
			{
				if(cur_xymove.remaining < 30)
				{
					while(the_route[route_pos].backmode == 0 && route_pos < the_route_len-1)
					{
						if( (sq(cur_x-the_route[route_pos+1].x)+sq(cur_y-the_route[route_pos+1].y) < sq(800) )
						    && check_direct_route_mm(cur_ang, cur_x, cur_y, the_route[route_pos+1].x, the_route[route_pos+1].y))
						{
							printf("Maneuver done; skipping point (%d, %d), going directly to (%d, %d)\n", the_route[route_pos].x,
							       the_route[route_pos].y, the_route[route_pos+1].x, the_route[route_pos+1].y);
							route_pos++;
						}
						else
						{
							break;
						}
					}
					id_cnt = 1;
					printf("Maneuver done, redo the waypoint, id=%d!\n", (id_cnt<<4) | ((route_pos)&0b1111));
					move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), cur_speedlim, 0);
					send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);

				}
			}
			else
			{
				if(cur_xymove.remaining < 250)
				{
					good_time_for_lidar_mapping = 1;
				}

				if(cur_xymove.remaining < the_route[route_pos].take_next_early)
				{
					maneuver_cnt = 0;
					if(route_pos < the_route_len-1)
					{
						route_pos++;

						// Check if we can skip some points:
						while(the_route[route_pos].backmode == 0 && route_pos < the_route_len-1)
						{
							if( (sq(cur_x-the_route[route_pos+1].x)+sq(cur_y-the_route[route_pos+1].y) < sq(800) )
							  && check_direct_route_mm(cur_ang, cur_x, cur_y, the_route[route_pos+1].x, the_route[route_pos+1].y))
							{
								printf("skipping point (%d, %d), going directly to (%d, %d)\n", the_route[route_pos].x,
								       the_route[route_pos].y, the_route[route_pos+1].x, the_route[route_pos+1].y);
								route_pos++;
							}
							else
							{
								break;
							}
						}
						printf("Take the next, id=%d!\n", (id_cnt<<4) | ((route_pos)&0b1111));
						move_to(the_route[route_pos].x, the_route[route_pos].y, the_route[route_pos].backmode, (id_cnt<<4) | ((route_pos)&0b1111), cur_speedlim, 0);
						send_info(the_route[route_pos].backmode?INFO_STATE_REV:INFO_STATE_FWD);
						micronavi_stops = 0;
					}
					else
					{
						printf("Done following the route.\n");
						send_info(INFO_STATE_IDLE);
						micronavi_stops = 0;
						do_follow_route = 0;
						route_finished_or_notfound = 1;
					}
				}
				else if(live_obstacle_checking_on)
				{
					// Check if obstacles have appeared in the map.

					static double prev_incr = 0.0;
					double stamp;
					if( (stamp=subsec_timestamp()) > prev_incr+0.10)
					{
						prev_incr = stamp;

						if(robot_pos_timestamp < stamp-0.20)
						{
							//printf("Skipping live obstacle checking due to stale robot pos.\n");
						}
						else
						{
							do_live_obstacle_checking();
						}
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

	printf("Set charger pos at ang=%d, x=%d, y=%d\n", cha_ang, cha_x, cha_y);
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
		printf("charger position retrieved from file: %d, %d --> %d, %d, ang=%d, fwd=%d\n", charger_first_x, charger_first_y, charger_second_x, charger_second_y, charger_ang, charger_fwd);
	}
}


int cal_x_d_offset = 0;
int cal_y_d_offset = 0;
float cal_x_offset = 40.0;
float cal_y_offset = 0.0;
float cal_x_sin_mult = 1.125;
float cal_y_sin_mult = 1.125;

#ifdef PULUTOF1
void request_tof_quit(void);
#endif

volatile int retval = 0;

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

	send_keepalive();
	daiju_mode(0);
	correct_robot_pos(0,0,0, pos_corr_id); // To set the pos_corr_id.
	turn_and_go_rel_rel(-5*ANG_1_DEG, 0, 25, 1);
	sleep(1);
	send_keepalive();
	turn_and_go_rel_rel(10*ANG_1_DEG, 0, 25, 1);
	sleep(1);
	send_keepalive();
	turn_and_go_rel_rel(-5*ANG_1_DEG, 50, 25, 1);
	sleep(1);
	send_keepalive();
	turn_and_go_rel_rel(0, -50, 25, 1);
	sleep(1);

	set_hw_obstacle_avoidance_margin(0);

	double chafind_timestamp = 0.0;
	int lidar_ignore_over = 0;
	while(1)
	{
		// Calculate fd_set size (biggest fd+1)
		int fds_size = 
#ifdef SIMULATE_SERIAL
			0;
#else		
			uart;
#endif
		if(tcp_listener_sock > fds_size) fds_size = tcp_listener_sock;
		if(tcp_client_sock > fds_size) fds_size = tcp_client_sock;
		if(STDIN_FILENO > fds_size) fds_size = STDIN_FILENO;
		fds_size+=1;


		fd_set fds;
		FD_ZERO(&fds);
#ifndef SIMULATE_SERIAL
		FD_SET(uart, &fds);
#endif
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

		static uint8_t pid_i_max = 30;
		static uint8_t pid_feedfwd = 30;
		static uint8_t pid_p = 100;
		static uint8_t pid_i = 20;
		static uint8_t pid_d = 20;

		if(FD_ISSET(STDIN_FILENO, &fds))
		{
			int cmd = fgetc(stdin);
			if(cmd == 'q')
			{
				retval = 0;
				break;
			}
			if(cmd == 'Q')
			{
				retval = 5;
				break;
			}
/*
			if(cmd == 'S')
			{
				save_robot_pos();
			}
			if(cmd == 's')
			{
				retrieve_robot_pos();
			}
*/
/*			if(cmd == 'c')
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
*/			if(cmd == '0')
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
				massive_search_area();
				printf("Requesting massive search.\n");
			}
			if(cmd == 'L')
			{
				conf_charger_pos();
			}
			if(cmd == 'l')
			{
				hw_find_charger();

//				read_charger_pos();
//				find_charger_state = 1;
			}
			if(cmd == 'v')
			{
				if(motors_on)
				{
					motors_on = 0;
					release_motors();
					printf("Robot is free to move manually.\n");
				}
				else
				{
					motors_on = 1;
					printf("Robot motors enabled again.\n");
				}
			}

			if(cmd == 'V')
			{
				verbose_mode = verbose_mode?0:1;
			}

#ifdef PULUTOF1
			if(cmd == 'z')
			{
				pulutof_decr_dbg();
			}
			if(cmd == 'x')
			{
				pulutof_incr_dbg();
			}
			if(cmd >= '1' && cmd <= '4')
			{
				pulutof_cal_offset(cmd-'1');
			}
#endif

/*			if(cmd >= '1' && cmd <= '9')
			{
				uint8_t bufings[3];
				bufings[0] = 0xd0 + cmd-'0';
				bufings[1] = 0;
				bufings[2] = 0xff;
				printf("Sending dev msg: %x\n", bufings[0]);
				send_uart(bufings, 3);				
			}
			if(cmd == 'A') {int tmp = (int)pid_i_max*5/4; if(tmp>255) tmp=255; pid_i_max=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'a') {int tmp = (int)pid_i_max*3/4; if(tmp<4) tmp=4;     pid_i_max=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'S') {int tmp = (int)pid_feedfwd*5/4; if(tmp>255) tmp=255; pid_feedfwd=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 's') {int tmp = (int)pid_feedfwd*3/4; if(tmp<3) tmp=4;     pid_feedfwd=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'D') {int tmp = (int)pid_p*5/4; if(tmp>255) tmp=255; pid_p=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'd') {int tmp = (int)pid_p*3/4; if(tmp<4) tmp=4;     pid_p=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'F') {int tmp = (int)pid_i*5/4; if(tmp>255) tmp=255; pid_i=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'f') {int tmp = (int)pid_i*3/4; if(tmp<4) tmp=4;     pid_i=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'G') {int tmp = (int)pid_d*5/4; if(tmp>255) tmp=255; pid_d=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'g') {int tmp = (int)pid_d*3/4; if(tmp<4) tmp=4;     pid_d=tmp; send_motcon_pid(pid_i_max, pid_feedfwd, pid_p, pid_i, pid_d);}
			if(cmd == 'z') {turn_and_go_rel_rel(0, 2000, 25, 1);}
			if(cmd == 'Z') {turn_and_go_rel_rel(0, -2000, 25, 1);}
*/
		}

#ifndef SIMULATE_SERIAL
		if(FD_ISSET(uart, &fds))
		{
			handle_uart();
		}
#endif

		if(tcp_client_sock >= 0 && FD_ISSET(tcp_client_sock, &fds))
		{
			int ret = handle_tcp_client();
			if(ret == TCP_CR_DEST_MID)
			{
				motors_on = 1;
				daiju_mode(0);

				printf("  ---> DEST params: X=%d Y=%d backmode=0x%02x\n", msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode);
				if(msg_cr_dest.backmode & 0b1000) // Pose
				{
					float ang = atan2(msg_cr_dest.y-cur_y, msg_cr_dest.x-cur_x);
					turn_and_go_abs_rel(RADTOANG32(ang), 0, cur_speedlim, 1);
				}
				else
					move_to(msg_cr_dest.x, msg_cr_dest.y, msg_cr_dest.backmode, 0, cur_speedlim, 1);
				find_charger_state = 0;
				lookaround_creep_reroute = 0;
				do_follow_route = 0;
				send_info(INFO_STATE_IDLE);

			}
			else if(ret == TCP_CR_ROUTE_MID)
			{
				printf("  ---> ROUTE params: X=%d Y=%d dummy=%d\n", msg_cr_route.x, msg_cr_route.y, msg_cr_route.dummy);

				motors_on = 1;
				daiju_mode(0);
				find_charger_state = 0;
				if(run_search(msg_cr_route.x, msg_cr_route.y, 0, 1) == 1)
				{
					printf("Routing fails in the start, daijuing for a while to get a better position.\n");
					daiju_mode(1);
					send_info(INFO_STATE_DAIJUING);
					printf("(Please retry after some time.)\n");
				}

			}
			else if(ret == TCP_CR_CHARGE_MID)
			{
				read_charger_pos();
				find_charger_state = 1;
			}
			else if(ret == TCP_CR_ADDCONSTRAINT_MID)
			{
				printf("  ---> ADD CONSTRAINT params: X=%d Y=%d\n", msg_cr_addconstraint.x, msg_cr_addconstraint.y);
				add_map_constraint(&world, msg_cr_addconstraint.x, msg_cr_addconstraint.y);
			}
			else if(ret == TCP_CR_REMCONSTRAINT_MID)
			{
				printf("  ---> REMOVE CONSTRAINT params: X=%d Y=%d\n", msg_cr_remconstraint.x, msg_cr_remconstraint.y);
				for(int xx=-2; xx<=2; xx++)
				{
					for(int yy = -2; yy<=2; yy++)
					{
						remove_map_constraint(&world, msg_cr_remconstraint.x + xx*40, msg_cr_remconstraint.y + yy*40);
					}
				}
			}
			else if(ret == TCP_CR_MODE_MID)			{
				printf("Request for MODE %d\n", msg_cr_mode.mode);
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
						find_charger_state = 0;
						lookaround_creep_reroute = 0;
						do_follow_route = 0;
						send_info(INFO_STATE_IDLE);
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
						find_charger_state = 0;
						lookaround_creep_reroute = 0;
						do_follow_route = 0;
						motors_on = 1;
						send_info(INFO_STATE_DAIJUING);
						daiju_mode(1);
						mapping_on = 0;
					} break;

					case 5:
					{
						stop_automapping();
						find_charger_state = 0;
						lookaround_creep_reroute = 0;
						do_follow_route = 0;
						send_info(INFO_STATE_IDLE);
						motors_on = 0;
						release_motors();
						mapping_on = 1;
					} break;

					case 6:
					{
						stop_automapping();
						find_charger_state = 0;
						lookaround_creep_reroute = 0;
						send_info(INFO_STATE_IDLE);
						do_follow_route = 0;
						motors_on = 0;
						release_motors();
						mapping_on = 0;
					} break;

					case 7:
					{
						conf_charger_pos();
					} break;

					case 8:
					{
						stop_automapping();
						find_charger_state = 0;
						lookaround_creep_reroute = 0;
						do_follow_route = 0;
						stop_movement();
						send_info(INFO_STATE_IDLE);
					} break;

					case 9:
					{
						
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
				printf("Manual OP %d\n", msg_cr_manu.op);
				switch(msg_cr_manu.op)
				{
					case MANU_FWD:
						turn_and_go_abs_rel(cur_ang, 100, 10, 1);
					break;
					case MANU_BACK:
						turn_and_go_abs_rel(cur_ang, -100, 10, 1);
					break;
					case MANU_LEFT:
						turn_and_go_abs_rel(cur_ang-10*ANG_1_DEG, 0, 10, 1);
					break;
					case MANU_RIGHT:
						turn_and_go_abs_rel(cur_ang+10*ANG_1_DEG, 0, 10, 1);
					break;
					default:
					break;
				}
			}		
			else if(ret == TCP_CR_MAINTENANCE_MID)
			{
				if(msg_cr_maintenance.magic == 0x12345678)
				{
					retval = msg_cr_maintenance.retval;
					break;
				}
				else
				{
					printf("WARN: Illegal maintenance message magic number 0x%08x.\n", msg_cr_maintenance.magic);
				}
			}		
			else if(ret == TCP_CR_SPEEDLIM_MID)
			{
				int new_lim = msg_cr_speedlim.speedlim_linear_fwd;
				printf("INFO: Speedlim msg %d\n", new_lim);
				if(new_lim < 1 || new_lim > MAX_CONFIGURABLE_SPEEDLIM)
					max_speedlim = DEFAULT_SPEEDLIM;
				else
					max_speedlim = new_lim;

				if(cur_speedlim > max_speedlim)
				{
					cur_speedlim = max_speedlim;
					limit_speed(cur_speedlim);
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
//			printf("Compass ang=%.1f deg\n", ANG32TOFDEG(cur_compass_ang));
//		}

		static int micronavi_stop_flags_printed = 0;

		if(cur_xymove.micronavi_stop_flags)
		{
			if(!micronavi_stop_flags_printed)
			{
				micronavi_stop_flags_printed = 1;
				printf("MCU-level micronavigation: STOP. Reason flags:\n");
				for(int i=0; i<32; i++)
				{
					if(cur_xymove.micronavi_stop_flags&(1UL<<i))
					{
						printf("bit %2d: %s\n", i, MCU_NAVI_STOP_NAMES[i]);
					}
				}

				printf("Actions being taken:\n");
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
				printf("Feedback module reported: %s\n", MCU_FEEDBACK_COLLISION_NAMES[stop_reason]);
				if(mapping_on)
				{
					map_collision_obstacle(&world, cur_ang, cur_x, cur_y, stop_reason, cur_xymove.stop_xcel_vector_valid,
						cur_xymove.stop_xcel_vector_ang_rad);
					if(do_follow_route)
					{
						int px, py, ox, oy;
						page_coords(cur_x, cur_y, &px, &py, &ox, &oy);

						for(int ix=-1; ix<=1; ix++)
						{
							for(int iy=-1; iy<=1; iy++)
							{
								gen_routing_page(&world, px+ix, py+iy, 0);
							}
						}
					}
				}
			}
		}
		else
			feedback_stop_flags_processed = 0;

		if(find_charger_state < 4)
			live_obstacle_checking_on = 1;
		else
			live_obstacle_checking_on = 0;

		if(find_charger_state == 1)
		{
			motors_on = 1;
			daiju_mode(0);
			if(run_search(charger_first_x, charger_first_y, 0, 1) != 0)
			{
				printf("Finding charger (first point) failed.\n");
				find_charger_state = 0;
			}
			else
				find_charger_state++;
		}
		else if(find_charger_state == 2)
		{
			if(!do_follow_route && !lookaround_creep_reroute)
			{
				if(sq(cur_x-charger_first_x) + sq(cur_y-charger_first_y) > sq(300))
				{
					printf("We are not at the first charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					send_info(INFO_STATE_THINK);
					printf("At first charger point, turning for charger.\n");
					turn_and_go_abs_rel(charger_ang, 0, 23, 1);
					find_charger_state++;
					chafind_timestamp = subsec_timestamp();

				}
			}
		}
		else if(find_charger_state == 3)
		{
			double stamp;
			if( (stamp=subsec_timestamp()) > chafind_timestamp+2.5)
			{
				send_info(INFO_STATE_THINK);

				chafind_timestamp = stamp;

				printf("Turned at first charger point, mapping lidars for exact pos.\n");

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
			if(lidar_ignore_over && subsec_timestamp() > chafind_timestamp+3.0)
			{
				printf("Going to second charger point.\n");
				send_info(INFO_STATE_FWD);
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
					printf("We are not at the second charger point, trying again.\n");
					find_charger_state = 1;
				}
				else
				{
					send_info(INFO_STATE_THINK);

					turn_and_go_abs_rel(charger_ang, charger_fwd, 20, 1);
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
				send_info(INFO_STATE_THINK);
				chafind_timestamp = stamp;
				turn_and_go_abs_rel(charger_ang, 0, 23, 1);
				find_charger_state++;
			}
		}
		else if(find_charger_state == 7)
		{
			double stamp;
			if( (stamp=subsec_timestamp()) > chafind_timestamp+1.5)
			{
				chafind_timestamp = stamp;
				send_info(INFO_STATE_THINK);
				printf("Requesting charger mount.\n");
				hw_find_charger();
				find_charger_state++;
			}
		}
		else if(find_charger_state == 8)
		{
			if(!pwr_status.charging && !pwr_status.charged)
			{
				if(subsec_timestamp() > chafind_timestamp+90.0)
				{
					printf("WARNING: Not charging (charger mount failure?). Retrying driving to charger.\n");
					find_charger_state = 1;
				}
			}
			else
			{
				send_info(INFO_STATE_CHARGING);
				find_charger_state = 0;
				printf("Robot charging succesfully.\n");
			}
		}

		route_fsm();
		autofsm();

#if 0 //def PULUTOF1
		{
			static double prev_incr = 0.0;
			double stamp;
			if( (stamp=subsec_timestamp()) > prev_incr+0.15)
			{
				prev_incr = stamp;

				extern int32_t tof3d_obstacle_levels[3];
				extern pthread_mutex_t cur_pos_mutex;
				int obstacle_levels[3];
				pthread_mutex_lock(&cur_pos_mutex);
				obstacle_levels[0] = tof3d_obstacle_levels[0];
				obstacle_levels[1] = tof3d_obstacle_levels[1];
				obstacle_levels[2] = tof3d_obstacle_levels[2];
				pthread_mutex_unlock(&cur_pos_mutex);

				if(obstacle_levels[2] > 100)
				{
					if(cur_speedlim > 18)
					{
						cur_speedlim = 18;
						limit_speed(cur_speedlim);
					}
				}
				else if(obstacle_levels[2] > 7)
				{
					if(cur_speedlim > 25)
					{
						cur_speedlim = 25;
						limit_speed(cur_speedlim);
					}
				}
				else if(obstacle_levels[1] > 70)
				{
					if(cur_speedlim > 25)
					{
						cur_speedlim = 25;
						limit_speed(cur_speedlim);
					}
				}
				else if(obstacle_levels[1] > 7)
				{
					if(cur_speedlim > 35)
					{
						cur_speedlim = 35;
						limit_speed(cur_speedlim);
					}
				}
				else if(obstacle_levels[0] > 20)
				{
					if(cur_speedlim > 42)
					{
						cur_speedlim = 42;
						limit_speed(cur_speedlim);
					}
				}
				else
				{
					if(cur_speedlim < max_speedlim)
					{
						cur_speedlim++;
						limit_speed(cur_speedlim);
					}
				}
			}
		}

#endif

#ifdef PULUTOF1

#ifdef PULUTOF1_GIVE_RAWS

		pulutof_frame_t* p_tof;
		if( (p_tof = get_pulutof_frame()) )
		{
			if(tcp_client_sock >= 0)
			{
#ifdef PULUTOF_EXTRA
				tcp_send_picture(p_tof->dbg_id, 2, 160, 60, p_tof->dbg);
#endif
				tcp_send_picture(100,           2, 160, 60, (uint8_t*)p_tof->depth);
#ifdef PULUTOF_EXTRA
				tcp_send_picture(110,           2, 160, 60, (uint8_t*)p_tof->uncorrected_depth);
#endif
			}

		}

#else
		tof3d_scan_t *p_tof;
		if( (p_tof = get_tof3d()) )
		{

			if(tcp_client_sock >= 0)
			{
				static int hmap_cnt = 0;
				hmap_cnt++;

				if(hmap_cnt >= 4)
				{
					//printf("Send hmap\n");
					tcp_send_hmap(TOF3D_HMAP_XSPOTS, TOF3D_HMAP_YSPOTS, p_tof->robot_pos.ang, p_tof->robot_pos.x, p_tof->robot_pos.y, TOF3D_HMAP_SPOT_SIZE, p_tof->objmap);
					//printf("Done\n");
					hmap_cnt = 0;
				}
			}

			static int32_t prev_x, prev_y, prev_ang;

			if(mapping_on && !pwr_status.charging && !pwr_status.charged)
			{
				if(p_tof->robot_pos.x != 0 || p_tof->robot_pos.y != 0 || p_tof->robot_pos.ang != 0)
				{
					int robot_moving = 0;
					if((prev_x != p_tof->robot_pos.x || prev_y != p_tof->robot_pos.y || prev_ang != p_tof->robot_pos.ang))
					{
						prev_x = p_tof->robot_pos.x; prev_y = p_tof->robot_pos.y; prev_ang = p_tof->robot_pos.ang;
						robot_moving = 1;
					}

					static int n_tofs_to_map = 0;
					static tof3d_scan_t* tofs_to_map[25];

					tofs_to_map[n_tofs_to_map] = p_tof;
					n_tofs_to_map++;

					if(n_tofs_to_map >= (robot_moving?3:20))
					{
						int32_t mid_x, mid_y;
						map_3dtof(&world, n_tofs_to_map, tofs_to_map, &mid_x, &mid_y);

						if(do_follow_route)
						{
							int px, py, ox, oy;
							page_coords(mid_x, mid_y, &px, &py, &ox, &oy);

							for(int ix=-1; ix<=1; ix++)
							{
								for(int iy=-1; iy<=1; iy++)
								{
									gen_routing_page(&world, px+ix, py+iy, 0);
								}
							}
						}

						n_tofs_to_map = 0;
					}
				}
			}

		}
#endif
#endif




		{
			static double prev_incr = 0.0;
			double stamp;
			if( (stamp=subsec_timestamp()) > prev_incr+0.15)
			{
				prev_incr = stamp;

				if(cur_speedlim < max_speedlim)
				{
					cur_speedlim++;
					//printf("cur_speedlim++ to %d\n", cur_speedlim);
					limit_speed(cur_speedlim);
				}

				if(cur_speedlim > max_speedlim)
				{
					cur_speedlim--;
					limit_speed(cur_speedlim);
				}
			}
		}

		lidar_scan_t* p_lid;

		if( (p_lid = get_significant_lidar()) || (p_lid = get_basic_lidar()) )
		{
			static int hwdbg_cnt = 0;
			hwdbg_cnt++;
			if(hwdbg_cnt > 0)
			{
				if(tcp_client_sock >= 0) tcp_send_hwdbg(hwdbg);
				hwdbg_cnt = 0;
			}

			static int lidar_send_cnt = 0;
			lidar_send_cnt++;
			if(lidar_send_cnt > 3)
			{
				if(tcp_client_sock >= 0) tcp_send_lidar_lowres(p_lid);
				lidar_send_cnt = 0;
			}

			static int lidar_ignore_cnt = 0;

			if(p_lid->id != pos_corr_id)
			{

				lidar_ignore_cnt++;

//				if(p_lid->significant_for_mapping) 
//				printf("Ignoring lidar scan with id=%d (significance=%d).\n", p_lid->id, p_lid->significant_for_mapping);

				if(lidar_ignore_cnt > 20)
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
				//printf("Got lidar scan. (%d)\n", p_lid->significant_for_mapping);

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
					}
					curpos_send_cnt = 0;
				}

				page_coords(p_lid->robot_pos.x, p_lid->robot_pos.y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_25pages(&world, idx_x, idx_y);

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
							if(n_lidars_to_map < 5)
							{
								printf("Got DISTORTED significant lidar scan, have too few lidars -> mapping queue reset\n");
								n_lidars_to_map = 0;
							}
							else
							{
								printf("Got DISTORTED significant lidar scan, running mapping early on previous images\n");
								int32_t da, dx, dy;
#ifdef PULUTOF1
								prevent_3dtoffing();
#endif
								map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
								INCR_POS_CORR_ID();
								correct_robot_pos(da, dx, dy, pos_corr_id);

								n_lidars_to_map = 0;
							}
						}
						else
						{
							//printf("Got significant(%d) lidar scan, adding to the mapping queue(%d).\n", p_lid->significant_for_mapping, n_lidars_to_map);
							lidars_to_map[n_lidars_to_map] = p_lid;

							n_lidars_to_map++;
							if((good_time_for_lidar_mapping && n_lidars_to_map > 5) || n_lidars_to_map > 12)
							{
								if(good_time_for_lidar_mapping) good_time_for_lidar_mapping = 0;
								int32_t da, dx, dy;
#ifdef PULUTOF1
								prevent_3dtoffing();
#endif
								map_lidars(&world, n_lidars_to_map, lidars_to_map, &da, &dx, &dy);
								INCR_POS_CORR_ID();
								correct_robot_pos(da, dx, dy, pos_corr_id);

								// keep a few old lidars:
								if(n_lidars_to_map > 12)
								{
									lidars_to_map[0] = lidars_to_map[9];
									lidars_to_map[1] = lidars_to_map[10];
									lidars_to_map[2] = lidars_to_map[11];
									lidars_to_map[3] = lidars_to_map[12];
									n_lidars_to_map = 4;
								}
								else
								{
									lidars_to_map[0] = lidars_to_map[n_lidars_to_map-2];
									lidars_to_map[1] = lidars_to_map[n_lidars_to_map-1];
									n_lidars_to_map = 2;
								}

							}
						}
					}
					else
					{
						n_lidars_to_map = 0;
					}

				}

			}

		}

		static int keepalive_cnt = 0;
		if(++keepalive_cnt > 500)
		{
			keepalive_cnt = 0;
			if(motors_on)
				send_keepalive();
			else
				release_motors();
		}


		sonar_point_t* p_son;
		if( (p_son = get_sonar()) )
		{
			if(tcp_client_sock >= 0) tcp_send_sonar(p_son);
			if(mapping_on)
				map_sonars(&world, 1, p_son);
		}

		static double prev_sync = 0;
		double stamp;

		double write_interval = 30.0;
		if(tcp_client_sock >= 0)
			write_interval = 7.0;

		if( (stamp=subsec_timestamp()) > prev_sync+write_interval)
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

#ifdef PULUTOF1
	request_tof_quit();
#endif

	return NULL;
}


#ifdef PULUTOF1
void* start_tof(void*);
#endif

int main(int argc, char** argv)
{
	pthread_t thread_main, thread_tof, thread_tof2;

	int ret;

	if( (ret = pthread_create(&thread_main, NULL, main_thread, NULL)) )
	{
		printf("ERROR: main thread creation, ret = %d\n", ret);
		return -1;
	}

#ifdef PULUTOF1
	if( (ret = pthread_create(&thread_tof, NULL, pulutof_poll_thread, NULL)) )
	{
		printf("ERROR: tof3d access thread creation, ret = %d\n", ret);
		return -1;
	}

	#ifndef PULUTOF1_GIVE_RAWS
		if( (ret = pthread_create(&thread_tof2, NULL, pulutof_processing_thread, NULL)) )
		{
			printf("ERROR: tof3d processing thread creation, ret = %d\n", ret);
			return -1;
		}
	#endif
#endif

	pthread_join(thread_main, NULL);

#ifdef PULUTOF1
	pthread_join(thread_tof, NULL);
	pthread_join(thread_tof2, NULL);
#endif

	return retval;
}
