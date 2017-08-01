#ifndef TOF3D_H
#define TOF3D_H

#define TOF3D_HMAP_SPOT_SIZE 40
#define TOF3D_HMAP_YSPOTS 36
#define TOF3D_HMAP_YMIDDLE 18
#define TOF3D_HMAP_XSPOTS 30


#define TOF3D_RING_BUF_LEN 32

#include "datatypes.h" // for pos_t

typedef struct
{
	pos_t  robot_pos;
	int8_t objmap[TOF3D_HMAP_YSPOTS*TOF3D_HMAP_XSPOTS];
} tof3d_scan_t;

#endif
