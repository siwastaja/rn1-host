#ifndef TOF3D_H
#define TOF3D_H

#define TOF3D_WALL           5    // Tall obstacle, count as wall
#define TOF3D_BIG_ITEM       4    // Large obstacle
#define TOF3D_SMALL_ITEM     3    // Small object. Could be a step, but most likely something to be avoided.
#define TOF3D_POSSIBLE_ITEM  2    // Possibility of a small step; could be a false positive, too.
#define TOF3D_SEEN           1    // Nothing special
#define TOF3D_POSSIBLE_DROP -1    // Small hole or drop. Could be a false positive, too.
#define TOF3D_DROP          -2    // Significant hole or drop.
#define TOF3D_UNSEEN         0


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
