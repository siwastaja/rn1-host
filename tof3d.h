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

#ifdef PULU1
	#define TOF3D_HMAP_YSPOTS 24
	#define TOF3D_HMAP_YMIDDLE 12
	#define TOF3D_HMAP_XSPOTS 20
#else
	#define TOF3D_HMAP_YSPOTS 36
	#define TOF3D_HMAP_YMIDDLE 18
	#define TOF3D_HMAP_XSPOTS 30
#endif

#define TOF3D_RING_BUF_LEN 32

#include "datatypes.h" // for pos_t

typedef struct
{
	pos_t  robot_pos;
	int8_t objmap[TOF3D_HMAP_YSPOTS*TOF3D_HMAP_XSPOTS];
} tof3d_scan_t;

#endif
