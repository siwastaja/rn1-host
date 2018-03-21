//#define SPI_PRINT_DBG

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



	Driver for SPI-connected PULUTOF 3D Time-of-Flight add-on

	For Raspberry Pi 3, make sure that:
	dtparam=spi=on     is in /boot/config.txt uncommented
	/dev/spidev0.0 should exist

	Also:
	/boot/cmdline.txt:  spidev.bufsiz=65536
	This often defaults to 4096, which is ridiculously too small.
	(65535 is hardware maximum for STM32 DMA transfer, so bigger transfers
	couldn't be easily utilized, anyway.)

*/

#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#include "pulutof.h"

#define PULUTOF_SPI_DEVICE "/dev/spidev0.0"


static int spi_fd;
static volatile int running = 1;

static const unsigned char spi_mode = SPI_MODE_0;
static const unsigned char spi_bits_per_word = 8;
static const unsigned int spi_speed = 32000000; // Hz

static int init_spi()
{
	spi_fd = open(PULUTOF_SPI_DEVICE, O_RDWR);

	if(spi_fd < 0)
	{
		printf("ERROR: Opening PULUTOF SPI device %s failed: %d (%s).\n", PULUTOF_SPI_DEVICE, errno, strerror(errno));
		return -1;
	}

	/*
		SPI_MODE_0 CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
		SPI_MODE_1 CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_2 CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_3 CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	*/

	/*
		Several code examples, (for example, http://www.raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface),
		have totally misunderstood what SPI_IOC_WR_* and SPI_IOC_RD_* mean. They are not different settings for SPI RX/TX respectively
		(that wouldn't make any sense: SPI by definition has synchronous RX and TX so clearly the settings are always the same). Instead,
		SPI_IOC_WR_* and SPI_IOC_RD_* write and read the driver settings, respectively, as explained in spidev documentation:
		https://www.kernel.org/doc/Documentation/spi/spidev

		Here, we just set what we need.
	*/

	if(ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_MODE failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_BITS_PER_WORD failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_MAX_SPEED_HZ failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	return 0;
}


static int deinit_spi()
{
	if(close(spi_fd) < 0)
	{
		printf("WARNING: Closing PULUTOF SPI devide failed: %d (%s).\n", errno, strerror(errno));
		return -1;
	}

	return 0;
}

volatile int dbg_id = 0;

void pulutof_decr_dbg()
{
	if(dbg_id) dbg_id--;
	printf("PULUTOF dbg_id=%d\n", dbg_id);
}

void pulutof_incr_dbg()
{
	dbg_id++;
	printf("PULUTOF dbg_id=%d\n", dbg_id);
}

#define PULUTOF_RINGBUF_LEN 16
volatile pulutof_frame_t pulutof_ringbuf[PULUTOF_RINGBUF_LEN];
volatile int pulutof_ringbuf_wr = 0;
volatile int pulutof_ringbuf_rd = 0;

#define TOF3D_RING_BUF_LEN 32

volatile tof3d_scan_t tof3ds[TOF3D_RING_BUF_LEN];
volatile int tof3d_wr;
volatile int tof3d_rd;

tof3d_scan_t* get_tof3d()
{
	if(tof3d_wr == tof3d_rd)
	{
		return 0;
	}
	
	tof3d_scan_t* ret = &tof3ds[tof3d_rd];
	tof3d_rd++; if(tof3d_rd >= TOF3D_RING_BUF_LEN) tof3d_rd = 0;
	return ret;
}


pulutof_frame_t* get_pulutof_frame()
{
	if(pulutof_ringbuf_wr == pulutof_ringbuf_rd)
	{
		return 0;
	}
	
	pulutof_frame_t* ret = &pulutof_ringbuf[pulutof_ringbuf_rd];
	pulutof_ringbuf_rd++; if(pulutof_ringbuf_rd >= PULUTOF_RINGBUF_LEN) pulutof_ringbuf_rd = 0;
	return ret;
}


static float x_angs[TOF_XS*TOF_YS];
static float y_angs[TOF_XS*TOF_YS];

#define GEOCAL_N_X 5
#define GEOCAL_N_Y 6

typedef struct
{
	int sens_x;
	int sens_y;
	float ang_x;
	float ang_y;
} geocal_point_t;

/*
	Pixel coordinates
*/
static const geocal_point_t lens_quadrant_coords[GEOCAL_N_Y+1][GEOCAL_N_X+1] =
{
	{ { 12,   2,   50.0, 25.0}, { 19,   1,   45.0, 25.0}, { 28,   0,   40.0, 25.0}, { 43,  -1,   30.0, 25.0}, { 62,  -2,   15.0, 25.0}, { 80,  -2,   0, 25.0} },
	{ { 11,   5,   50.0, 22.5}, { 19,   4,   45.0, 22.5}, { 27,   3,   40.0, 22.5}, { 42,   2,   30.0, 22.5}, { 62,   1,   15.0, 22.5}, { 80,   1,   0, 22.5} },
	{ { 11,   8,   50.0, 20.0}, { 19,   6,   45.0, 20.0}, { 27,   6,   40.0, 20.0}, { 42,   5,   30.0, 20.0}, { 62,   4,   15.0, 20.0}, { 80,   4,   0, 20.0} },
	{ { 10,  10,   50.0, 17.5}, { 18,   9,   45.0, 17.5}, { 27,   9,   40.0, 17.5}, { 42,   8,   30.0, 17.5}, { 62,   8,   15.0, 17.5}, { 80,   8,   0, 17.5} },
	{ { 10,  13,   50.0, 15.0}, { 18,  12,   45.0, 15.0}, { 27,  12,   40.0, 15.0}, { 42,  11,   30.0, 15.0}, { 62,  11,   15.0, 15.0}, { 80,  11,   0, 15.0} },
	{ { 10,  19,   50.0, 10.0}, { 18,  18,   45.0, 10.0}, { 26,  18,   40.0, 10.0}, { 42,  17,   30.0, 10.0}, { 62,  17,   15.0, 10.0}, { 80,  17,   0, 10.0} },
	{ { 10,  29,   50.0,  0.0}, { 18,  29,   45.0,  0.0}, { 26,  29,   40.0,  0.0}, { 42,  29,   30.0,  0.0}, { 62,  29,   15.0,  0.0}, { 80,  29,   0,  0.0} }
};

static float x_angs[TOF_XS*TOF_YS];
static float y_angs[TOF_XS*TOF_YS];


/*
	Sensor mount position 1:
	 _ _
	| | |
	| |L|
	|O|L|
	| |L|
	|_|_|  (front view)

	Sensor mount position 2:
	 _ _
	| | |
	|L| |
	|L|O|
	|L| |
	|_|_|  (front view)

	Sensor mount position 3:

	-------------
	|  L  L  L  |
	-------------
	|     O     |
	-------------

	Sensor mount position 4:

	-------------
	|     O     |
	-------------
	|  L  L  L  |
	-------------
*/

typedef struct
{
	int mount_mode;             // mount position 1,2,3 or 4
	float x_rel_robot;          // zero = robot origin. Positive = robot front (forward)
	float y_rel_robot;          // zero = robot origin. Positive = to the right of the robot
	float ang_rel_robot;        // zero = robot forward direction. positive = ccw
	float vert_ang_rel_ground;  // zero = looks directly forward. positive = looks up. negative = looks down
	float z_rel_ground;         // sensor height from the ground	
} sensor_mount_t;

#define NUM_PULUTOFS 4

#ifdef PULUTOF_ROBOT_SER_1_TO_4  // Retrofitted sensors
static const sensor_mount_t sensor_mounts[NUM_PULUTOFS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0: Left rear      */ { 2,  -276,  -233, DEGTORAD(      90), DEGTORAD(  0), 227 },
 /*1: Right rear     */ { 1,  -276,   233, DEGTORAD(     270), DEGTORAD(  0), 227 },
 /*2: Right front    */ { 2,   154,   164, DEGTORAD(       0), DEGTORAD(  0), 228 },
 /*3: Left front     */ { 1,   154,  -164, DEGTORAD(       0), DEGTORAD(  0), 228 }
};
#endif

#ifdef PULUTOF_ROBOT_SER_5_UP
static const sensor_mount_t sensor_mounts[NUM_PULUTOFS] =
{          //      mountmode    x     y       hor ang           ver ang      height    
 /*0: Left rear      */ { 2,  -183,  -218, DEGTORAD(90  + 30), DEGTORAD( 0), 234 },
 /*1: Right rear     */ { 1,  -183,   218, DEGTORAD(270 - 30), DEGTORAD( 0), 234 },
 /*2: Right front    */ { 1,   138,   115, DEGTORAD(     -10), DEGTORAD(10), 170 },
 /*3: Left front     */ { 2,   138,  -115, DEGTORAD(     +10), DEGTORAD(10), 170 }
};
#endif

int32_t hmap_accum[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
int16_t hmap_nsamples[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
int16_t hmap[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
static int32_t hmap_calib_accum[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
static int16_t hmap_calib_cnt[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
static int16_t hmap_calib[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];
int16_t hmap_avgd[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];

static void distances_to_objmap(pulutof_frame_t *in)
{
	int sidx = in->sensor_idx;
	if(sidx > NUM_PULUTOFS-1)
	{
		printf("WARNING: distances_to_objmap: illegal sensor idx coming from hw.\n");
		return;
	}

	/*
		for converting to absolute world coordinates, if that's needed in the future:
	
	float robot_ang = ANG32TORAD(in->robot_pos.ang);
	float robot_x = in->robot_pos.x;
	float robot_y = in->robot_pos.y;

	float sensor_ang = robot_ang + sensor_mounts[sidx].ang_rel_robot;
	float sensor_x = robot_x + cos(robot_ang)*sensor_mounts[sidx].x_rel_robot;
	float sensor_y = robot_y + sin(robot_ang)*sensor_mounts[sidx].y_rel_robot;
	*/

	float sensor_ang = sensor_mounts[sidx].ang_rel_robot;
	float sensor_x = sensor_mounts[sidx].x_rel_robot;
	float sensor_y = sensor_mounts[sidx].y_rel_robot;
	float sensor_yang = sensor_mounts[sidx].vert_ang_rel_ground;
	float sensor_z = sensor_mounts[sidx].z_rel_ground;
	


	for(int pyy = 1; pyy < TOF_YS-1; pyy++)
//	for(int pyy = 25; pyy < 35; pyy++)
	{
		for(int pxx = 1; pxx < TOF_XS-1; pxx++)
//		for(int pxx = 75; pxx < 85; pxx++)
		{
			int n_valids = 0;
			int avg = 0;
			for(int dyy=-1; dyy<=1; dyy++)
			{
				for(int dxx=-1; dxx<=1; dxx++)
				{
					int dist = in->depth[(pyy+dyy)*TOF_XS+(pxx+dxx)];
					if(dist != 0)
					{
						n_valids++;
						avg += dist;
					}
				}
			}

			if(n_valids > 6)
			{
				avg /= n_valids;
				int n_conforming = 0;
				int avg_conforming = 0;
				int cumul_dxx = 0, cumul_dyy = 0;
				for(int dyy=-1; dyy<=1; dyy++)
				{
					for(int dxx=-1; dxx<=1; dxx++)
					{
						int dist = in->depth[(pyy+dyy)*TOF_XS+(pxx+dxx)];
						if(dist != 0 && dist > avg-350 && dist < avg+350)
						{
							n_conforming++;
							avg_conforming += dist;
							cumul_dxx += dxx;
							cumul_dyy += dyy;
						}
					}
				}

				if(n_conforming > 4)
				{
					int py, px;
					if(cumul_dxx < -2) px = pxx-1; else if(cumul_dxx > 2) px = pxx+1; else px = pxx;
					if(cumul_dyy < -2) py = pyy-1; else if(cumul_dyy > 2) py = pyy+1; else py = pyy;

					float hor_ang, ver_ang;

					switch(sensor_mounts[sidx].mount_mode)
					{
						case 1: 
						hor_ang = -1*y_angs[py*TOF_XS+px];
						ver_ang = x_angs[py*TOF_XS+px];
						break;

						case 2: 
						hor_ang = y_angs[py*TOF_XS+px];
						ver_ang = -1*x_angs[py*TOF_XS+px];
						break;

						case 3: // direction in which the original geometrical calibration was calculated in
						hor_ang = -1*x_angs[py*TOF_XS+px];
						ver_ang = y_angs[py*TOF_XS+px];
						break;

						case 4: // Same as 3, but upside down
						hor_ang = x_angs[py*TOF_XS+px];
						ver_ang = -1*y_angs[py*TOF_XS+px];
						break;

						default: printf("ERROR: illegal mount_mode in sensor mount table.\n"); return;
					}

					// From spherical to cartesian coordinates

					float d = (float)avg_conforming/(float)n_conforming;

				//	d *= 0.84; // don't understand why, yet. Seems to be a very consistent error!
				//	d *= 0.90;

					float x = d * /*sin*/cos(ver_ang + sensor_yang) * cos(hor_ang + sensor_ang) + sensor_x;
					float y = -1* (d * /*sin*/cos(ver_ang + sensor_yang) * sin(hor_ang + sensor_ang)) + sensor_y;
					float z = d * /*cos*/sin(ver_ang + sensor_yang) + sensor_z;

					int xspot = (int)(x / (float)TOF3D_HMAP_SPOT_SIZE) + TOF3D_HMAP_XMIDDLE;
					int yspot = (int)(y / (float)TOF3D_HMAP_SPOT_SIZE) + TOF3D_HMAP_YMIDDLE;

					//printf("DIST = %.0f  x=%.0f  y=%.0f  z=%.0f  xspot=%d  yspot=%d  ver_ang=%.2f  sensor_yang=%.2f  hor_ang=%.2f  sensor_ang=%.2f\n", d, x, y, z, xspot, yspot, ver_ang, sensor_yang, hor_ang, sensor_ang); 

					if(xspot < 0 || xspot >= TOF3D_HMAP_XSPOTS || yspot < 0 || yspot >= TOF3D_HMAP_YSPOTS)
					{
						//ignored++;
						continue;
					}

/*					int zi = z;
					if(zi > -2000 && zi < 2000)
					{
						if(zi > hmap_accum[xspot][yspot])
							hmap_accum[xspot][yspot] = zi;
						hmap_nsamples[xspot][yspot]++;
					}
*/

					uint8_t new_val = 0;
					if( z < -150.0)
						new_val = TOF3D_BIG_DROP;
					else if(z < -100.0)
						new_val = TOF3D_SMALL_DROP;
					else if(z < 100.0)
						new_val = TOF3D_FLOOR;
					else if(z < 150.0)
						new_val = TOF3D_THRESHOLD;
					else if(z < 265.0)
						new_val = TOF3D_SMALL_ITEM;
					else if(z < 295.0)
						new_val = TOF3D_WALL;
					else if(z < 2100.0)
						new_val = TOF3D_BIG_ITEM;

					if(new_val > tof3ds[tof3d_wr].objmap[yspot*TOF3D_HMAP_XSPOTS+xspot])
						tof3ds[tof3d_wr].objmap[yspot*TOF3D_HMAP_XSPOTS+xspot] = new_val;
				}
				
			}
			
		}
	}
}

static int calibrating = 0;
static int calib_sensor_idx = 0;

static void process_pulutof_frame(pulutof_frame_t *in);

void* pulutof_processing_thread()
{
	printf("tof3d: opening tof_zcalib.raw Z axis calibration file for read.\n");
	FILE* floor = fopen("/home/hrst/rn1-host/tof_zcalib.raw", "r");
	if(!floor)
	{
		printf("WARNING: Couldn't open tof_zcalib.raw for read. Floor calibration is disabled.\n");
	}
	else
	{
		fread(hmap_calib, 2, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS, floor);
		fclose(floor);
	}

	while(running)
	{
		pulutof_frame_t* p_tof;
		if( (p_tof = get_pulutof_frame()) )
		{
			process_pulutof_frame(p_tof);
		}
		else
		{
			usleep(5000);
		}

	}

	return NULL;

}
static void process_objmap();
extern pthread_mutex_t cur_pos_mutex;
extern int32_t cur_ang, cur_x, cur_y;
int32_t tof3d_obstacle_levels[3];


static void process_pulutof_frame(pulutof_frame_t *in)
{
	static int running_ok = 0;
	static int prev_sidx = -1;

	int sidx = in->sensor_idx;

	if(sidx > NUM_PULUTOFS-1)
	{
		printf("WARNING:process_pulutof_frame: illegal sensor idx coming from hw.\n");
		return;
	}

	int expected_sidx = prev_sidx+1; if(expected_sidx >= NUM_PULUTOFS) expected_sidx = 0;

	if(running_ok && expected_sidx != sidx)
	{
		printf("WARNING:process_pulutof_frame: unexpected sensor idx %d, previous was %d, was expecting %d. Ignoring until 0\n", sidx, prev_sidx, expected_sidx);
		running_ok = 0;
	}

	if(sidx == 0)
	{
		running_ok = 1;
//		for(int xx=0; xx < TOF3D_HMAP_XSPOTS; xx++) { for(int yy=0; yy < TOF3D_HMAP_YSPOTS; yy++)
//			{hmap_accum[xx][yy] = -9999; hmap_nsamples[xx][yy] = 0; hmap_avgd[xx][yy] = 0;} }

		memset(tof3ds[tof3d_wr].objmap, 0, 1*TOF3D_HMAP_YSPOTS*TOF3D_HMAP_XSPOTS);
	}

	if(running_ok)
	{
		if(calibrating)
		{
			if(calib_sensor_idx == sidx)
			{
				distances_to_objmap(in);
//				process_objmap();
			}
		}
		else
		{
//			if(sidx==2)
			distances_to_objmap(in);


			if(sidx == 2)
			{
				pthread_mutex_lock(&cur_pos_mutex);
				tof3ds[tof3d_wr].robot_pos.ang = cur_ang;
				tof3ds[tof3d_wr].robot_pos.x = cur_x;
				tof3ds[tof3d_wr].robot_pos.y = cur_y;
				pthread_mutex_unlock(&cur_pos_mutex);
			}

			if(sidx == NUM_PULUTOFS-1)
			{
				// Got all sensors: the objmap accumulation done, process it.
//				process_objmap();
				tof3d_wr++; if(tof3d_wr >= TOF3D_RING_BUF_LEN) tof3d_wr = 0;

			}
		}
	}

	prev_sidx = sidx;
}

void run_3dtof_floor_calibration()
{
	
	calibrating = 1;
}



static void process_objmap()
{
	for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
	{
		for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
		{
			if(hmap_nsamples[sx][sy] > 0)
				tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx] = hmap_accum[sx][sy]/20;
			else
				tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx] = 0;
/*
			if(hmap_nsamples[sx][sy] > 0)
			{
				hmap[sx][sy] = hmap_accum[sx][sy]; //hmap_accum[sx][sy]/hmap_nsamples[sx][sy];
				if(calibrating)
				{
					hmap_calib_accum[sx][sy] += hmap[sx][sy];
					hmap_calib_cnt[sx][sy]++;
				}
				else
				{
					if(hmap_calib[sx][sy] == -9999)
						hmap[sx][sy] = -9999;
					else
						hmap[sx][sy] -= hmap_calib[sx][sy];
				}

			}
			else
				hmap[sx][sy] = -9999;

*/
		}
	}


#if 0
	#define NUM_CALIB_CNT 400
	if(calibrating)
	{
		static int calib_cnt = 0;
		calib_cnt++;
		if(calib_cnt > NUM_CALIB_CNT)
		{
			if(calib_sensor_idx < NUM_PULUTOFS-1)
			{
				printf("\n------------------------------------------------------------------------------------------------------\n");
				printf("INFO: Calibration done on sensor %d. Please align the robot for sensor %d and rerun calibration command.\n", calib_sensor_idx, calib_sensor_idx+1);
				printf("------------------------------------------------------------------------------------------------------\n\n");
				calib_cnt = 0;
				calibrating = 0;
				calib_sensor_idx++;
				return;
			}
			else
			{

				FILE* floor = fopen("tof_zcalib.raw", "w");

				int32_t avg_accum = 0;
				int avg_cnt = 0;

				for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
				{
					for(int sx = 0; sx < TOF3D_HMAP_YSPOTS; sx++)
					{
						if(hmap_calib_cnt[sx][sy] < 10)
							hmap_calib[sx][sy] = -9999;
						else
						{
							hmap_calib[sx][sy] = hmap_calib_accum[sx][sy] / hmap_calib_cnt[sx][sy];
							if(hmap_calib_cnt[sx][sy] < 50)
								hmap_calib[sx][sy] /= 2; // to avoid overcorrecting in tricky cases

							avg_accum += hmap_calib[sx][sy];
							avg_cnt++;
						}
					}
				}

				int32_t avg = avg_accum / avg_cnt;
				int body_ignores = 0;

				int16_t hmap_calib_copy[TOF3D_HMAP_XSPOTS][TOF3D_HMAP_YSPOTS];

				memcpy(hmap_calib_copy, hmap_calib, sizeof(hmap_calib));

				// Generate robot body ignores:
				for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
				{
					for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
					{
						if(hmap_calib_copy[sx][sy] > avg + 70 || hmap_calib_copy[sx][sy] == -9999)
						{
							body_ignores++;
							hmap_calib[sx+0][sy+0] = -9999;
							if(sy>0) hmap_calib[sx+0][sy-1] = -9999;
							if(sy<TOF3D_HMAP_YSPOTS-1) hmap_calib[sx+0][sy+1] = -9999;
							if(sx < TOF3D_HMAP_XSPOTS-1)
							{
								hmap_calib[sx+1][sy+0] = -9999;
								if(sy>0) hmap_calib[sx+1][sy-1] = -9999;
								if(sy<TOF3D_HMAP_YSPOTS-1) hmap_calib[sx+1][sy+1] = -9999;
							}
							if(sx > 0)
							{
								hmap_calib[sx-1][sy+0] = -9999;
								if(sy>0) hmap_calib[sx-1][sy-1] = -9999;
								if(sy<TOF3D_HMAP_YSPOTS-1) hmap_calib[sx-1][sy+1] = -9999;
							}
						}
					}
				}
				fwrite(hmap_calib, 2, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS, floor);
				fclose(floor);

				FILE* floor_csv = fopen("tof_zcalib_stats.csv", "w");
				for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
				{
					for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
					{
						if(hmap_calib[sx][sy] == -9999)
							;//	fprintf(floor_csv, "");
						else
							fprintf(floor_csv, "%d", hmap_calib[sx][sy]);
						fprintf(floor_csv, ",");
					}
					fprintf(floor_csv, "\n");
				}
				fprintf(floor_csv, "\n");
				fprintf(floor_csv, "frames=,%d,spots=,%d,out_of=,%d\n", NUM_CALIB_CNT, avg_cnt, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS);
				fprintf(floor_csv, "avg_corr=,%d,ignored=,%d,(+neighbors)\n", avg, body_ignores);

				printf("------------------------------------------------------------------------\n");
				printf("------------------------- CALIBRATION FINISHED -------------------------\n");
				printf("------------------------------------------------------------------------\n");
				printf(" %d frames    %d spots used (out of %d)\n", NUM_CALIB_CNT, avg_cnt, TOF3D_HMAP_XSPOTS*TOF3D_HMAP_YSPOTS);
				printf(" avg corr = %d     ignored %d locations (+ their neighbors) as robot body\n", avg, body_ignores);
				printf("------------------------------------------------------------------------\n");
				printf("Calibration saved in tof_zcalib.raw. Also generated tof_zcalib_stats.csv\n");
				printf("       Please quit and rerun rn1host to load the calibration\n");
				printf("------------------------------------------------------------------------\n");
				calibrating = 0;
				calib_sensor_idx = 0;

			}
		}
		return;
	}


	// ------------ GENERATE SOFT AVGD MAP ---------

//	for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
//		for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
//			hmap_avgd[sx][sy] = 0;

	for(int sx = 3; sx < TOF3D_HMAP_XSPOTS-3; sx++)
	{
		for(int sy = 3; sy < TOF3D_HMAP_YSPOTS-3; sy++)
		{
			int nsamp = 0;
			int acc = 0;

			for(int ix=-6; ix<=6; ix++)
			{
				for(int iy=-6; iy<=6; iy++)
				{
					if(sx+ix < 0 || sx+ix > TOF3D_HMAP_XSPOTS-1 || sy+iy < 0 || sy+iy > TOF3D_HMAP_YSPOTS-1)
						continue;

					if(hmap[sx+ix][sy+iy] > -50 && hmap[sx+ix][sy+iy] < 50)
					{
						nsamp++;
						acc += hmap[sx+ix][sy+iy];
					}
				}
			}

			if(nsamp < 40 || hmap[sx][sy] < -1000)
				hmap_avgd[sx][sy] = 0;
			else
				hmap_avgd[sx][sy] = hmap[sx][sy] - acc/nsamp;
		}
	}

	// ------------ GENERATE OBJMAP BASED ON HMAP and AVGD -------------


	int obst_cnts[3] = {0,0,0}; // [0] = very far counts, [1] somewhat near, [2] very near.

//	for(int sx = 0; sx < TOF3D_HMAP_XSPOTS; sx++)
//		for(int sy = 0; sy < TOF3D_HMAP_YSPOTS; sy++)
//			tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx] = hmap[sx][sy]<-1000?-100:(hmap[sx][sy]/20);

	for(int sx = 1; sx < TOF3D_HMAP_XSPOTS-1; sx++)
	{
		for(int sy = 1; sy < TOF3D_HMAP_YSPOTS-1; sy++)
		{
			int nearness = 0;
			if(sx < (500)/TOF3D_HMAP_SPOT_SIZE &&
			   sy > TOF3D_HMAP_YMIDDLE - (280)/TOF3D_HMAP_SPOT_SIZE &&
			   sy < TOF3D_HMAP_YMIDDLE + (280)/TOF3D_HMAP_SPOT_SIZE)
				nearness = 2;
			else if(sx < (1000)/TOF3D_HMAP_SPOT_SIZE &&
			   sy > TOF3D_HMAP_YMIDDLE - (360)/TOF3D_HMAP_SPOT_SIZE &&
			   sy < TOF3D_HMAP_YMIDDLE + (360)/TOF3D_HMAP_SPOT_SIZE)
				nearness = 1;


			int8_t val = TOF3D_SEEN;
			if(hmap[sx][sy] < -998 || hmap_avgd[sx][sy] < -998)
				val = TOF3D_UNSEEN;
			else if(hmap[sx][sy] < -200 || hmap_avgd[sx][sy] < -110)
			{
				val = TOF3D_DROP;

				obst_cnts[nearness] += 8;
			}
			else if(hmap[sx][sy] < -170 || hmap_avgd[sx][sy] < -80)
			{
				val = TOF3D_POSSIBLE_DROP;
				obst_cnts[nearness] += 1;
			}
			else if(hmap[sx][sy] > 200 || hmap_avgd[sx][sy] > 120)
			{
				val = TOF3D_WALL;
				obst_cnts[nearness] += 8;
			}
			else if(hmap[sx][sy] > 100 || hmap_avgd[sx][sy] > 40)
			{
				val = TOF3D_BIG_ITEM;
				obst_cnts[nearness] += 8;
			}
			else if(hmap[sx][sy] > 90 || hmap_avgd[sx][sy] > 25)
			{
				val = TOF3D_SMALL_ITEM;
				obst_cnts[nearness] += 4;
			}
			else if(hmap[sx][sy] > 80 || hmap_avgd[sx][sy] > 15)
			{
				val = TOF3D_POSSIBLE_ITEM;
				obst_cnts[nearness] += 1;
			}
			else
			{
				int diffsum = 0;

				for(int ix=-1; ix<=1; ix++)
				{
					for(int iy=-1; iy<=1; iy++)
					{
						if(hmap_avgd[sx+ix][sy+iy] < 15)
							diffsum += hmap_avgd[sx+ix][sy+iy];
					}
				}

				if(diffsum > 90)
				{
					val = TOF3D_SMALL_ITEM;
					obst_cnts[nearness] += 4;
				}
				else if(diffsum > 70)
				{
					val = TOF3D_POSSIBLE_ITEM;
					obst_cnts[nearness] += 1;
				}
			}

			tof3ds[tof3d_wr].objmap[sy*TOF3D_HMAP_XSPOTS+sx] = val;
		}
	}

	pthread_mutex_lock(&cur_pos_mutex);
	tof3d_obstacle_levels[0] = obst_cnts[0];
	tof3d_obstacle_levels[1] = obst_cnts[1];
	tof3d_obstacle_levels[2] = obst_cnts[2];
	pthread_mutex_unlock(&cur_pos_mutex);


#endif


	tof3d_wr++; if(tof3d_wr >= TOF3D_RING_BUF_LEN) tof3d_wr = 0;
}

static void print_table()
{
	for(int yy = 0; yy < TOF_YS; yy++)
	{
		for(int xx=150; xx < 160; xx++)
		{
			printf("(%5.1f, %5.1f) ", x_angs[yy*TOF_XS+xx], y_angs[yy*TOF_XS+xx]);
		}
		printf("\n");
	}

	printf("\n");
	printf("\n");
}

static void outp_ang(int px, int py, float ax, float ay)
{
//	printf("(%3d, %3d)=(%5.1f, %5.1f)\n", px, py, ax, ay);

	if(py < 0 || py > TOF_YS-1 || px < 0 || px > TOF_XS-1)
		return;

	x_angs[py*TOF_XS+px] = DEGTORAD(ax);
	y_angs[py*TOF_XS+px] = DEGTORAD(ay);
}

static void gen_ang_tables()
{
	for(int i=0; i<TOF_XS*TOF_YS; i++)
	{
		x_angs[i] = 999.0; // uninit marker
		y_angs[i] = 999.0;
	}

	for(int calyy=0; calyy<GEOCAL_N_Y+1; calyy++)
	{
		// Left half, extrapolate the first X record
		{
			float dpx = lens_quadrant_coords[calyy][1].sens_x - lens_quadrant_coords[calyy][0].sens_x;
			float dpy = lens_quadrant_coords[calyy][1].sens_y - lens_quadrant_coords[calyy][0].sens_y;
			float dax = lens_quadrant_coords[calyy][1].ang_x - lens_quadrant_coords[calyy][0].ang_x;
			float ax_per_px = dax/dpx;
			float py_per_px = dpy/dpx;
			float cur_ax = -1*lens_quadrant_coords[calyy][0].ang_x + ax_per_px*lens_quadrant_coords[calyy][0].sens_x;
			float cur_ay = -1*lens_quadrant_coords[calyy][0].ang_y;
			float cur_py = lens_quadrant_coords[calyy][0].sens_y - py_per_px*lens_quadrant_coords[calyy][0].sens_x;
			for(int pxx=0; pxx<lens_quadrant_coords[calyy][0].sens_x; pxx++)
			{
				outp_ang(pxx, (int)(cur_py+0.5), cur_ax, cur_ay);
				cur_py += py_per_px;
				cur_ax -= ax_per_px;
			}
		}
		for(int calxx=0; calxx<GEOCAL_N_X; calxx++)
		{
			// Interpolate between the two
			float dpx = lens_quadrant_coords[calyy][calxx+1].sens_x - lens_quadrant_coords[calyy][calxx+0].sens_x;
			float dpy = lens_quadrant_coords[calyy][calxx+1].sens_y - lens_quadrant_coords[calyy][calxx+0].sens_y;
			float dax = lens_quadrant_coords[calyy][calxx+1].ang_x - lens_quadrant_coords[calyy][calxx+0].ang_x;
			float ax_per_px = dax/dpx;
			float py_per_px = dpy/dpx;
			float cur_ax = -1*lens_quadrant_coords[calyy][calxx].ang_x;
			float cur_ay = -1*lens_quadrant_coords[calyy][calxx].ang_y;
			float cur_py = lens_quadrant_coords[calyy][calxx].sens_y;

			for(int pxx=lens_quadrant_coords[calyy][calxx].sens_x; pxx<lens_quadrant_coords[calyy][calxx+1].sens_x; pxx++)
			{
				outp_ang(pxx, (int)(cur_py+0.5), cur_ax, cur_ay);
				cur_py += py_per_px;
				cur_ax -= ax_per_px;
			}
			
		}

		outp_ang(lens_quadrant_coords[calyy][GEOCAL_N_X].sens_x, lens_quadrant_coords[calyy][GEOCAL_N_X].sens_y, lens_quadrant_coords[calyy][GEOCAL_N_X].ang_x, -1*lens_quadrant_coords[calyy][GEOCAL_N_X].ang_y);

	}

	// Fill the missing cells in Y direction
	// Extrapolates at the start if necessary; but expects the last cell (at TOF_XS/2-1) to be populated.
	for(int pxx=0; pxx<TOF_XS/2+1; pxx++)
	{
		float prev_ax, prev_ay;
		int prev_at = 999;
		for(int pyy=0; pyy < TOF_YS/2; pyy++)
		{
			if(x_angs[pyy*TOF_XS+pxx] != 999.0)
			{
				if(prev_at == 999 && pyy > 0) // need to extrapolate - there were empty spots before this
				{
					// Find next:
					int next_py = pyy+1;
					while(x_angs[next_py*TOF_XS+pxx] == 999.0) next_py++;

					float first_ax = x_angs[pyy*TOF_XS+pxx];
					float first_ay = y_angs[pyy*TOF_XS+pxx];
					float second_ax = x_angs[next_py*TOF_XS+pxx];
					float second_ay = y_angs[next_py*TOF_XS+pxx];
					float dax_per_step = (second_ax - first_ax) / (next_py - pyy);
					float day_per_step = (second_ay - first_ay) / (next_py - pyy);
					float cur_ax = first_ax - dax_per_step*pyy; 
					float cur_ay = first_ay - day_per_step*pyy;

					for(int iy=0; iy<pyy; iy++)
					{
						x_angs[iy*TOF_XS+pxx] = cur_ax;
						y_angs[iy*TOF_XS+pxx] = cur_ay;
						cur_ax += dax_per_step;
						cur_ay += day_per_step;
					}
				}

				if(prev_at != 999) // interpolate
				{
					float now_ax = x_angs[pyy*TOF_XS+pxx];
					float now_ay = y_angs[pyy*TOF_XS+pxx];
					float dax_per_step = (prev_ax - now_ax) / (prev_at - pyy);
					float day_per_step = (prev_ay - now_ay) / (prev_at - pyy);
					float cur_ax = prev_ax;
					float cur_ay = prev_ay;
					for(int iy=prev_at+1; iy<pyy; iy++)
					{
						cur_ax += dax_per_step;
						cur_ay += day_per_step;
						x_angs[iy*TOF_XS+pxx] = cur_ax;
						y_angs[iy*TOF_XS+pxx] = cur_ay;
					}
				}

				prev_ax = x_angs[pyy*TOF_XS+pxx];
				prev_ay = y_angs[pyy*TOF_XS+pxx];
				prev_at = pyy;
			}
		}
	}

	// Fill all four quadrants by mirroring
	// Fill bottom left
	for(int pxx=0; pxx<TOF_XS/2+1; pxx++)
	{
		for(int pyy=0; pyy < TOF_YS/2; pyy++)
		{
			int out_pyy = TOF_YS-pyy-2;
			x_angs[out_pyy*TOF_XS+pxx] = x_angs[pyy*TOF_XS+pxx];
			y_angs[out_pyy*TOF_XS+pxx] = -1*y_angs[pyy*TOF_XS+pxx];
		}
		// Extrapolate the last line:
		
		x_angs[(TOF_YS-1)*TOF_XS+pxx] = x_angs[(TOF_YS-2)*TOF_XS+pxx] - (x_angs[(TOF_YS-3)*TOF_XS+pxx] - x_angs[(TOF_YS-2)*TOF_XS+pxx]);
		y_angs[(TOF_YS-1)*TOF_XS+pxx] = y_angs[(TOF_YS-2)*TOF_XS+pxx] - (y_angs[(TOF_YS-3)*TOF_XS+pxx] - y_angs[(TOF_YS-2)*TOF_XS+pxx]);
	}

	// Fill the right half

	for(int pyy=0; pyy<TOF_YS; pyy++)
	{
		for(int pxx=0; pxx < TOF_XS/2; pxx++)
		{
			int out_pxx = TOF_XS-pxx-0;
			x_angs[pyy*TOF_XS+out_pxx] = -1*x_angs[pyy*TOF_XS+pxx];
			y_angs[pyy*TOF_XS+out_pxx] = y_angs[pyy*TOF_XS+pxx];	
		}
	}

	//print_table();

}

/*
	As you should know, SPI is a "show yours, I'll show mine" protocol. As a master, we'll only send something, and get something back.
	Unfortunately, Raspberry Pi can only be a master, in SPI terms.
	However, the PULUTOF in kind of a "real" master, because it decides when to image the frames, and will do it regardless of what we are
	doing on the Raspberry - the data is anyway needed for low-level obstacle avoidance, completely independent of the Raspi.

	poll_availability commands a small (five-byte) transfer, which will only return the first status byte. If we are happy with the status (i.e., there is
	data available), we can go on with the full transfer. This status byte further gives insight how long to wait before next polling is needed.

	When poll_availability informs there is a frame available, read_frame can be called - it assumes the data is indeed available and performs a full
	frame read (if the data is not available - the status byte clearly tells this, but you still read it, you'll hit an old frame, or
	possibly a corrupted combination of two frames).
*/


static uint8_t txbuf[65536];

static int poll_availability()
{
	txbuf[4] = dbg_id&0xff;	
	struct spi_ioc_transfer xfer;
	struct response { uint32_t header; uint8_t status;} response;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	//xfer.tx_buf left at 0 - documented spidev feature to send out zeroes - we don't have anything to send, just want to get what the sensor wants to send us!
	xfer.tx_buf = txbuf;
	xfer.rx_buf = &response;
	xfer.len = sizeof response;
	xfer.cs_change = 0; // deassert chip select after the transfer

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}

	if(response.header != 0x11223344 || response.status == 0)
	{
		printf("ERROR: Illegal response in poll_availability: header=0x%08x  status=%d\n", response.header, response.status);
		return -1;
	}
	//printf("status=%d\n", response.status);
	return response.status;
}
//#define SPI_PRINT_DBG

static int read_frame()
{
	txbuf[4] = dbg_id&0xff;	
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.

	//xfer.tx_buf left at 0 - documented spidev feature to send out zeroes - we don't have anything to send, just want to get what the sensor wants to send us!
	xfer.tx_buf = txbuf;
	xfer.rx_buf = &pulutof_ringbuf[pulutof_ringbuf_wr];
	xfer.len = sizeof(pulutof_frame_t);
	xfer.cs_change = 0; // deassert chip select after the transfer

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}

#ifdef SPI_PRINT_DBG
	printf("Frame (sensor_idx= %d) read ok, timing:\n", pulutof_ringbuf[pulutof_ringbuf_wr].sensor_idx);
	for(int i=0; i<24; i++)
	{
		printf("%d:%.1f ", i, (float)pulutof_ringbuf[pulutof_ringbuf_wr].timestamps[i]/10.0);
	}
	printf("\n");
	printf("Time deltas to:\n");
	for(int i=1; i<24; i++)
	{
		printf(">%d:%.1f ", i, (float)(pulutof_ringbuf[pulutof_ringbuf_wr].timestamps[i]-pulutof_ringbuf[pulutof_ringbuf_wr].timestamps[i-1])/10.0);
	}
	printf("\n");
	printf("dbg_i32:\n");
	for(int i=0; i<8; i++)
	{
		printf("[%d] %11d  ", i, pulutof_ringbuf[pulutof_ringbuf_wr].dbg_i32[i]);
	}
	printf("\n");
	printf("\n");
#endif

	int ret = pulutof_ringbuf[pulutof_ringbuf_wr].status;

	pulutof_ringbuf_wr++; if(pulutof_ringbuf_wr >= PULUTOF_RINGBUF_LEN) pulutof_ringbuf_wr = 0;

	return ret;
}

void request_tof_quit()
{
	running = 0;
}

void pulutof_cal_offset(uint8_t idx)
{
	printf("Requesting offset calib\n");
	struct spi_ioc_transfer xfer;
	struct cmd { uint32_t header; uint8_t sensor_idx;} cmd;

	cmd.header = 0xca0ff5e7;
	cmd.sensor_idx = idx;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	xfer.tx_buf = &cmd;
	xfer.rx_buf = NULL;
	xfer.len = sizeof cmd;
	xfer.cs_change = 0; // deassert chip select after the transfer

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return;
	}

}


void* pulutof_poll_thread()
{
	gen_ang_tables();
	init_spi();
	while(running)
	{
		int next = pulutof_ringbuf_wr+1; if(next >= PULUTOF_RINGBUF_LEN) next = 0;
		if(next == pulutof_ringbuf_rd)
		{
			printf("WARNING: PULUTOF ringbuf overflow prevented, ignoring images...\n");
			usleep(250000);
			continue;
		}


		int avail = poll_availability();

		if(avail < 0)
		{
#ifdef SPI_PRINT_DBG
		//	printf("Sleeping 2 s\n");
#endif
			sleep(2);
			continue;
		}

		if(avail < 250)
		{
#ifdef SPI_PRINT_DBG
		//	printf("Sleeping %d ms\n", avail);
#endif
			usleep(1000*avail);
			continue;
		}

		read_frame();


		usleep(1000);
	}
	deinit_spi();

	return NULL;
}

