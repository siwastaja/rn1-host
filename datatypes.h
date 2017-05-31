#ifndef DATATYPES_H
#define DATATYPES_H

#include <stdint.h>

#define ANG_2_5_DEG   29826162
#define ANG_1_DEG     11930465
#define ANG_0_5_DEG    5965232
#define ANG_0_25_DEG   2982616
#define ANG_0_125_DEG  1491308
#define ANG_0_1_DEG    1193047
#define ANG_0_05_DEG    596523


typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // in mm
	int32_t y;
} pos_t;

typedef struct
{
	int valid;
	int32_t x;   // in mm
	int32_t y;
} point_t;

#define LIDAR_SCAN_POINTS 360

typedef struct
{
	int significant_for_mapping;
	int is_invalid; // May be distorted due to excessive robot acceleration (collision, drop, etc.)
	pos_t robot_pos;
	point_t scan[LIDAR_SCAN_POINTS];
} lidar_scan_t;

typedef struct
{
	point_t scan[3];
} sonar_scan_t;

extern int32_t hwdbg[10];

typedef struct
{
	int bat_mv;
	int bat_percentage;
	int charging;
	int charged;
} pwr_status_t;

extern pwr_status_t pwr_status;

#endif
