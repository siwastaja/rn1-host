
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h> // for sleep
#include <pthread.h> // for mutexing current coords.

#include "mapping.h"
#include "datatypes.h"
#include "uart.h"

#include "../rn1-brain/comm.h" // For the convenient 7-bit data handling macros.
#define I14x2_I16(msb,lsb) ((int16_t)( ( ((uint16_t)(msb)<<9) | ((uint16_t)(lsb)<<2) ) ))


#define SIGNIFICANT_LIDAR_RING_BUF_LEN 32
#define LIDAR_RING_BUF_LEN 16
#define SONAR_RING_BUF_LEN 16

int lidar_wr = 0;
int lidar_rd = 0;
int significant_lidar_wr = 0;
int significant_lidar_rd = 0;
int sonar_wr = 0;
int sonar_rd = 0;

lidar_scan_t* latest_lidar;
lidar_scan_t lidars[LIDAR_RING_BUF_LEN];
lidar_scan_t significant_lidars[SIGNIFICANT_LIDAR_RING_BUF_LEN];
sonar_scan_t sonars[SONAR_RING_BUF_LEN];
xymove_t cur_xymove;

int32_t hwdbg[10];

lidar_scan_t* get_basic_lidar()
{
	if(lidar_wr == lidar_rd)
	{
		return 0;
	}
	
	lidar_scan_t* ret = &lidars[lidar_rd];
	lidar_rd++; if(lidar_rd >= LIDAR_RING_BUF_LEN) lidar_rd = 0;
	return ret;
}

lidar_scan_t* get_significant_lidar()
{
	if(significant_lidar_wr == significant_lidar_rd)
	{
		return 0;
	}
	
	lidar_scan_t* ret = &significant_lidars[significant_lidar_rd];
	significant_lidar_rd++; if(significant_lidar_rd >= SIGNIFICANT_LIDAR_RING_BUF_LEN) significant_lidar_rd = 0;
	return ret;
}


sonar_scan_t* get_sonar()
{
	if(sonar_wr == sonar_rd)
	{
		return 0;
	}
	
	sonar_scan_t* ret = &sonars[sonar_rd];
	sonar_rd++; if(sonar_rd >= SONAR_RING_BUF_LEN) sonar_rd = 0;
	return ret;
}

pwr_status_t pwr_status;


extern int32_t cur_ang, cur_x, cur_y;
pthread_mutex_t cur_pos_mutex = PTHREAD_MUTEX_INITIALIZER;
int update_robot_pos(int32_t ang, int32_t x, int32_t y)
{
	static int error_cnt = 0;
	if(   x <= -1*(MAP_PAGE_W_MM*(MAP_W/2-1)) || x >= MAP_PAGE_W_MM*(MAP_W/2-1)
	   || y <= -1*(MAP_PAGE_W_MM*(MAP_W/2-1)) || y >= MAP_PAGE_W_MM*(MAP_W/2-1) )
	{
		printf("ERROR: Illegal x,y mm coords (coming from HW) in update_robot_pos (ang=%d, x=%d, y=%d)\n", ang, x, y);

		error_cnt++;
		if(error_cnt > 10)
		{
			printf("ERROR: Illegal coords seem to be stuck. Giving up.\n");
			exit(1);
		}

		return -1;
	}
	if(error_cnt) error_cnt--;

	//printf("."); fflush(stdout);

	//printf("write %d, %d, %d\n", ang, x, y);
	pthread_mutex_lock(&cur_pos_mutex);
	cur_ang = ang; cur_x = x; cur_y = y;
	pthread_mutex_unlock(&cur_pos_mutex);

	return 0;
}

int parse_uart_msg(uint8_t* buf, int len)
{
	switch(buf[0])
	{
		case 0x84:
		{
			/*
			 Lidar-based 2D MAP on uart:

			num_bytes
			 1	uint8 start byte
			 1	uint7 status
			 1      uint7 id
			 2	int14 cur_ang (at the middle point of the lidar scan)  (not used for turning the image, just to include robot coords)
			 5	int32 cur_x   ( " " )
			 5	int32 cur_y   ( " " )
			 1	int7  correction return value
			 2	int14 ang_corr (for information only)
			 2	int14 x_corr (for information only)
			 2	int14 y_corr (for information only) 
			1440	360 * point
				  2	int14  x referenced to cur_x
				  2	int14  y referenced to cur_y

				Total: 1461
				Time to tx at 115200: ~130 ms

			*/

			int is_significant = buf[1]&0b11;

			if(is_significant)
			{
				int next = significant_lidar_wr+1; if(next >= SIGNIFICANT_LIDAR_RING_BUF_LEN) next = 0;
				if(next == significant_lidar_rd)
				{
					printf("WARNING: lidar ring buffer overrun prevented by ignoring lidar scan (significant scan).\n");
					break;
				}
			}
			else
			{
				int next = lidar_wr+1; if(next >= LIDAR_RING_BUF_LEN) next = 0;
				if(next == lidar_rd)
				{
					printf("WARNING: lidar ring buffer overrun prevented by ignoring lidar scan (basic scan).\n");
					break;
				}
			}

			lidar_scan_t* lid = is_significant?&significant_lidars[significant_lidar_wr]:&lidars[lidar_wr];
			latest_lidar = lid;
			lid->is_invalid = (buf[1]&4)?1:0;
			lid->significant_for_mapping = is_significant;
			lid->id = buf[2];
			lid->robot_pos.ang = (I7I7_U16_lossy(buf[3], buf[4]))<<16;
			int mid_x = lid->robot_pos.x = I7x5_I32(buf[5],buf[6],buf[7],buf[8],buf[9]);
			int mid_y = lid->robot_pos.y = I7x5_I32(buf[10],buf[11],buf[12],buf[13],buf[14]);

			if(update_robot_pos(lid->robot_pos.ang, mid_x, mid_y) < 0)
			{
//				printf("UART frame:");
//				for(int i = 0; i < len; i++) printf(" %02x", buf[i]);
//				printf("\n");
				break;
			}

			for(int i = 0; i < 360; i++)
			{
				int x = I14x2_I16(buf[22+4*i+0], buf[22+4*i+1])>>2;
				int y = I14x2_I16(buf[22+4*i+2], buf[22+4*i+3])>>2;
				if(x != 0 && y != 0)
				{
					lid->scan[i].x = x + mid_x;
					lid->scan[i].y = y + mid_y;
					lid->scan[i].valid = 1;
				}
				else
				{
					lid->scan[i].valid = 0;
				}
			}

			if(is_significant)
			{
				significant_lidar_wr++; if(significant_lidar_wr >= SIGNIFICANT_LIDAR_RING_BUF_LEN) significant_lidar_wr = 0;
			}
			else
			{
				lidar_wr++; if(lidar_wr >= LIDAR_RING_BUF_LEN) lidar_wr = 0;
			}
		}
		break;

		case 0x85:
		{
			/*
				Sonar-based 2D map
			*/
			sonars[sonar_wr].robot_pos.x = I7x5_I32(buf[2],buf[3],buf[4],buf[5],buf[6]);
			sonars[sonar_wr].robot_pos.y = I7x5_I32(buf[7],buf[8],buf[9],buf[10],buf[11]);

			for(int i = 0; i < 3; i++)
			{
				sonars[sonar_wr].scan[i].valid = (buf[1]&(1<<i))?1:0;
				sonars[sonar_wr].scan[i].x = I7x5_I32(buf[12+10*i],buf[13+10*i],buf[14+10*i],buf[15+10*i],buf[16+10*i]);
				sonars[sonar_wr].scan[i].y = I7x5_I32(buf[17+10*i],buf[18+10*i],buf[19+10*i],buf[20+10*i],buf[21+10*i]);
			}

			sonar_wr++; if(sonar_wr >= SONAR_RING_BUF_LEN) sonar_wr = 0;

		}
		break;

		case 0xa0: // current coords without lidar image
		{
			if(update_robot_pos((I7I7_U16_lossy(buf[2], buf[3]))<<16, 
				I7x5_I32(buf[4],buf[5],buf[6],buf[7],buf[8]),
				I7x5_I32(buf[9],buf[10],buf[11],buf[12],buf[13])) < 0)
			{
//				printf("UART frame:");
//				for(int i = 0; i < len; i++) printf(" %02x", buf[i]);
//				printf("\n");
				break;
			}

		}
		break;

		case 0xa2:
		{
			pwr_status.charging = buf[1]&1;
			pwr_status.charged = buf[1]&2;
			pwr_status.bat_mv = I7I7_U16_lossy(buf[2], buf[3]);
			pwr_status.bat_percentage = buf[4];
		}
		break;

		case 0xa3:
		{
			extern int32_t cur_compass_ang;
			extern int compass_round_active;
			compass_round_active = buf[1];
			cur_compass_ang = I7I7_U16_lossy(buf[2], buf[3])<<16;

//			printf("INFO: cur_compass_ang = %6.1fdeg  %s\n", ANG32TOFDEG(cur_compass_ang), compass_round_active?"CALIBRATING":"");
		}
		break;

		case 0xa5:
		{
			cur_xymove.status = buf[1];
			cur_xymove.id = buf[2];
			cur_xymove.remaining = I7I7_U16_lossy(buf[3], buf[4]);
			cur_xymove.micronavi_stop_flags = I7x5_I32(buf[5],buf[6],buf[7],buf[8],buf[9]);
			cur_xymove.micronavi_action_flags = I7x5_I32(buf[10],buf[11],buf[12],buf[13],buf[14]);
			cur_xymove.feedback_stop_flags = buf[15];
			cur_xymove.stop_xcel_vector_valid = buf[16] || buf[17] || buf[18] || buf[19];
			if(cur_xymove.stop_xcel_vector_valid)
			{
				int x = I7I7_U16_lossy(buf[16], buf[17]);
				int y = I7I7_U16_lossy(buf[18], buf[19]);
				float ang = atan2(y, x) - M_PI/2.0;
				if(ang < 0.0) ang += 2.0*M_PI;
				if(ang < 0.0) ang += 2.0*M_PI;
//				printf("Stop vector (reason %d): x=%d, y=%d, ang=%.1f deg\n", cur_xymove.feedback_stop_flags, x, y, RADTODEG(ang));
				cur_xymove.stop_xcel_vector_ang_rad = ang;
			}
		}
		break;

		case 0xd2:
		{
			for(int i=0; i<10; i++)
			{
				hwdbg[i] = I7x5_I32(buf[i*5+1],buf[i*5+2],buf[i*5+3],buf[i*5+4],buf[i*5+5]);
			}
		}
		break;
	}

	return 0;
}

void send_keepalive()
{
	uint8_t buf[3] = {0x8f, 42, 0xff};
	send_uart(buf, 3);
}

void release_motors()
{
	uint8_t buf[3] = {0x8f, 0, 0xff};
	send_uart(buf, 3);
}

void move_to(int32_t x, int32_t y, int8_t backmode, int id, int speedlimit, int accurate_turn)
{
	uint8_t buf[16];

	if(id < 0 || id > 127)
	{
		printf("ERROR: Invalid move_to id %d\n", id);
		id = 0;
	}

	printf("INFO: Move to (%d, %d), backmode=%d, id=%d\n", x,y,backmode,id);

	buf[0] = 0x82;
	buf[1] = I32_I7_4(x);
	buf[2] = I32_I7_3(x);
	buf[3] = I32_I7_2(x);
	buf[4] = I32_I7_1(x);
	buf[5] = I32_I7_0(x);
	buf[6] = I32_I7_4(y);
	buf[7] = I32_I7_3(y);
	buf[8] = I32_I7_2(y);
	buf[9] = I32_I7_1(y);
	buf[10] = I32_I7_0(y);
	buf[11] = ((uint8_t)backmode)&0x7f;
	buf[12] = ((uint8_t)id)&0x7f;
	buf[13] = speedlimit&0x7f;
	buf[14] = accurate_turn;
	buf[15] = 0xff;
	send_uart(buf, 16);
}

void turn_and_go(int32_t ang_abs, int fwd_rel, int speedlimit, int accurate_turn)
{
	uint8_t buf[8];

	printf("INFO: Turn & go %d, %d\n", ang_abs, fwd_rel);

	buf[0] = 0x83;
	buf[1] = I16_MS(ang_abs>>16);
	buf[2] = I16_LS(ang_abs>>16);
	buf[3] = I16_MS(fwd_rel);
	buf[4] = I16_LS(fwd_rel);
	buf[5] = speedlimit&0x7f;
	buf[6] = accurate_turn;
	buf[7] = 0xff;
	send_uart(buf, 8);
}

void stop_movement()
{
	uint8_t buf[8];

	printf("INFO: stop_movement()\n");

	buf[0] = 0x84;
	buf[1] = 0;
	buf[2] = 0;
	send_uart(buf, 3);
}


void correct_robot_pos(int32_t da, int32_t dx, int32_t dy, int id)
{
	dx<<=2;
	dy<<=2;

	if(dx < -32767 || dx > 32767 || dy < -32767 || dy > 32767 || id < 0 || id > 127)
	{
		printf("ERROR: out of range coords or id in correct_robot_pos()\n");
		return;
	}

	da *= -1; // Robot angles are opposite to those of trigonometric funtions.

/*	if(da == 0 && dx == 0 && dy == 0)
	{
		printf("INFO: No position correction needed.\n");
		return;
	}
*/
//	printf("INFO: Correcting robot pos by %d, %d, %d, with id %d\n", da>>16, dx, dy, id);

	uint8_t buf[9];

	buf[0] = 0x89;
	buf[1] = I16_MS(da>>16);
	buf[2] = I16_LS(da>>16);
	buf[3] = I16_MS((int16_t)dx);
	buf[4] = I16_LS((int16_t)dx);
	buf[5] = I16_MS((int16_t)dy);
	buf[6] = I16_LS((int16_t)dy);
	buf[7] = id;
	buf[8] = 0xff;

	send_uart(buf, 9);
}

void set_robot_pos(int32_t na, int32_t nx, int32_t ny)
{
	printf("INFO: Setting robot pos to ang=%d, x=%d, y=%d\n", na>>16, nx, ny);

	uint8_t buf[14];

	buf[0] = 0x8a;
	buf[1] = I16_MS(na>>16);
	buf[2] = I16_LS(na>>16);
	buf[3] = I32_I7_4(nx);
	buf[4] = I32_I7_3(nx);
	buf[5] = I32_I7_2(nx);
	buf[6] = I32_I7_1(nx);
	buf[7] = I32_I7_0(nx);
	buf[8] = I32_I7_4(ny);
	buf[9] = I32_I7_3(ny);
	buf[10] = I32_I7_2(ny);
	buf[11] = I32_I7_1(ny);
	buf[12] = I32_I7_0(ny);
	buf[13] = 0xff;

	send_uart(buf, 14);
}


void set_hw_obstacle_avoidance_margin(int mm)
{
	uint8_t buf[3];

	int cm = mm/10;
	if(cm < 0) cm = 0;
	else if(cm > 100) cm = 100;

//	printf("INFO: set_hw_obstacle_avoidance_margin to %d cm\n", cm);

	buf[0] = 0x88;
	buf[1] = cm;
	buf[2] = 0xff;

	send_uart(buf, 3);
}

void do_compass_round()
{
	uint8_t buf[3];

	buf[0] = 0x91;
	buf[1] = 0;
	buf[2] = 0xff;

	send_uart(buf, 3);
}

void hw_find_charger()
{
	uint8_t buf[3];

	buf[0] = 0x87;
	buf[1] = 0;
	buf[2] = 0xff;

	send_uart(buf, 3);
}

// Last resort when routefinding is stuck; robot randomly goes wherever it can.
void daiju_mode(int on)
{
	set_hw_obstacle_avoidance_margin(0);

	uint8_t buf[3];

	buf[0] = 0x86;
	buf[1] = on?5:0;
	buf[2] = 0xff;

	send_uart(buf, 3);
}
