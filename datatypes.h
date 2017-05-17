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
	int status;
	pos_t pos;
	int16_t scan[360];
} lidar_scan_t;

#define LIDAR_STATUS_SYNCED_IMAGES 0b1100

#endif
