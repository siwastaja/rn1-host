#ifndef DATATYPES_H
#define DATATYPES_H

#include <stdint.h>

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

typedef struct
{
	int significant_for_mapping;
	pos_t robot_pos;
	point_t scan[360];
} lidar_scan_t;

typedef struct
{
	point_t scan[3];
} sonar_scan_t;

extern int32_t hwdbg[10];

#endif
