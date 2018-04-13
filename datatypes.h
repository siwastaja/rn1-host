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

#ifndef DATATYPES_H
#define DATATYPES_H

#include <math.h>
#ifndef M_PI
#define M_PI 3.141592653589793238
#endif

#include <stdint.h>

#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047
#define ANG_0_05_DEG    596523

#define ANG32TORAD(x) ( ((float)((uint32_t)(x)))/683565275.576432)
#define ANG32TOFDEG(x) ( ((float)((uint32_t)(x)))/11930464.7111111)
#define RADTODEG(x) ((x)*(360.0/(2.0*M_PI)))
#define DEGTORAD(x) ((x)*((2.0*M_PI)/360.0))
#define RADTOANG32(x) ( (int32_t)((((float)(x)) / (2.0*M_PI)) * 4294967296.0))

typedef struct __attribute__((packed))
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // in mm
	int32_t y;
} pos_t;

typedef struct __attribute__((packed))
{
	int valid;
	int32_t x;   // in mm
	int32_t y;
} point_t;

#define MAX_LIDAR_POINTS 720

typedef struct
{
	int filtered;
	int significant_for_mapping;
	int is_invalid; // May be distorted due to excessive robot acceleration (collision, drop, etc.)
	int id; // id can be updated with the correct position message; this way we know when the scan has recent coordinate update done or not.
	pos_t robot_pos;
	int n_points;
	point_t scan[MAX_LIDAR_POINTS];
} lidar_scan_t;

typedef struct
{
	int32_t x;
	int32_t y;
	int16_t z;
	int8_t c;
} sonar_point_t;


typedef struct
{
	int32_t x;
	int32_t y;
	int16_t z;
} xyz_t;


extern int32_t hwdbg[10];

typedef struct
{
	int bat_mv;
	int bat_percentage;
	int cha_mv;
	int charging;
	int charged;
} pwr_status_t;

typedef struct
{
	int status;
	int id;
	int remaining;
	uint32_t micronavi_stop_flags;
	uint32_t micronavi_action_flags;
	uint32_t feedback_stop_flags;
	int stop_xcel_vector_valid;
	float stop_xcel_vector_ang_rad;
} xymove_t;

extern pwr_status_t pwr_status;
extern xymove_t cur_xymove;

typedef struct __attribute__((packed))
{
	int16_t first_movement_needed; // distance the robot needed to go fwd or back as the very first operation. 0 if within tolerances. in mm.
	uint8_t turning_passes_needed; // optical positioning needed to move the robot this many passes without needing to back of / go forward again (adjusting angle was enough alone)
	uint8_t vexling_passes_needed; // optical positioning needed to move the robot this many passes, doing a back-off-go-forward pass.
	uint8_t accepted_pos;          // 1, if optical positioning succesful. 0 if failed there.
	int16_t dist_before_push;      // after succesful optical positioning, the measured distance to the charger right before the push. in mm.
	uint8_t result;                // 100 = success. Others = failure.
} chafind_results_t;

extern chafind_results_t chafind_results;


#define MAP_SIGNIFICANT_IMGS     1
#define MAP_SEMISIGNIFICANT_IMGS 2
extern int map_significance_mode;

typedef enum 
{
	INFO_STATE_UNDEF = -1,
	INFO_STATE_IDLE = 0,
	INFO_STATE_THINK = 1,
	INFO_STATE_FWD = 2,
	INFO_STATE_REV = 3,
	INFO_STATE_LEFT = 4,
	INFO_STATE_RIGHT = 5,
	INFO_STATE_CHARGING = 6,
	INFO_STATE_DAIJUING = 7
} info_state_t;

#endif
