#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>
#include "datatypes.h"
#include "tof3d.h"

#define UNIT_FREE	0
#define UNIT_ITEM           (1<<0)	// Small obstacle, detected by sonars or bumping into it
#define UNIT_WALL           (1<<1)	// Obstacle seen by the lidar.
#define UNIT_INVISIBLE_WALL (1<<2)  // Only found out by collision
#define UNIT_3D_WALL        (1<<3)  // Wall seen by 3DTOF, can't be removed by lidar.
#define UNIT_DROP           (1<<4)
#define UNIT_DBG            (1<<6)
#define UNIT_MAPPED         (1<<7)  // We have seen this area.

#define CONSTRAINT_FORBIDDEN 	(1<<0)	// "Don't go here" unit

#define PLUS_SAT_255(x) {if((x)<255) (x)++;}
#define MINUS_SAT_0(x) {if((x)>0) (x)--;}


#define MM_TO_UNIT(x) ((x)/MAP_UNIT_W + MAP_MIDDLE_UNIT)

/*
Map unit is a 40mm*40mm area. Memory usage is carefully considered, because the world is a 2D map of the map units.
40mm*40mm was selected as a good compromise between accuracy (i.e., considering that the 500mm wide robot can maneuver through
tight passages, which would be wasteful if the unit was larger) and memory/disk usage.
*/

typedef struct __attribute__ ((packed))
{
	uint8_t result;   	// Mapping result decided based on all available data.
	uint8_t latest;  	// Mapping result based on last scan.

	uint8_t timestamp;	// Latest time scanned
	uint8_t num_visited;    // Incremented when lidar is mapped with this robot coord. Saturated at 255.

	uint8_t num_seen;  	// Number of times mapped. Saturated at 255.
	uint8_t num_obstacles;  // "is an obstacle" BY LIDAR counter. Every time mapped, ++ if obstacle, -- if not. Saturated at 255.

	uint8_t constraints;
	uint8_t num_3d_obstacles; // ++ if 3D_WALL, DROP, or ITEM. Set to 0 if those are removed.
} map_unit_t;


/*
Map page is a fixed 256*256 * 40mm*40mm = 10.24m*10.24m area.

At all times, the following principles apply:
- There are 9 map pages (3*3) in the memory
- The robot is always located in the middle page.

To satisfy these conditions, map pages are stored/retrieved to/from the disk every time the robot crosses the map border.
*/

#define MAP_UNIT_W 40  // in mm
#define MAP_PAGE_W 256 // in map_units
#define MAP_PAGE_W_MM (MAP_UNIT_W * MAP_PAGE_W)  // convenience define


/*
Quick map only holds the mapping result byte, and with half*half resolution; 1 map page will be 16Kbytes instead of 0.5Mbytes.
*/

#define QMAP_UNIT_W 80 // in mm
#define QMAP_PAGE_W 128
#define QMAP_PAGE_W_MM (QMAP_UNIT_W * QMAP_PAGE_W)


typedef struct
{
	map_unit_t units[MAP_PAGE_W][MAP_PAGE_W];
} map_page_t;


typedef struct
{
	uint8_t units[QMAP_PAGE_W][QMAP_PAGE_W];
} qmap_page_t;


/*
	Routing pages (for optimization purposes only) use single bits to denote forbidden areas, so
	that 32-bit wide robot shapes can be compared against hits efficiently. For the same reason,
	one extra uint32 block is included on the bottom (positive) end.
*/
typedef union
{
	uint32_t obst_u32[MAP_PAGE_W][MAP_PAGE_W/32 + 1];
} routing_page_t;


/*
world_t is one continuously mappable entity. There can be several worlds, but the worlds cannot overlap;
in case they would, they should be combined.

When the software cannot decide which world it's in, we create a new, empty world. If we figure out it matches
an earlier world, we will combine them.

To make things simple, we statically allocate for the map page pointers. Map pages themselves are dynamically allocated.

256*256 map pages will limit the maximum world size to 2.62 km * 2.62 km. On a 64-bit system, storing pointers to map pages
and qmap pages, 1Mbyte is required for the pointers. NULL pointer means the data is not loaded in memory.

*/

#define MAP_W 256
#define MAP_MIDDLE_PAGE (MAP_W/2)

#define MAP_MIDDLE_UNIT (MAP_PAGE_W * MAP_MIDDLE_PAGE)

typedef struct
{
	uint32_t id;

	map_page_t*  pages[MAP_W][MAP_W];
	uint8_t changed[MAP_W][MAP_W];
	qmap_page_t* qpages[MAP_W][MAP_W];
	routing_page_t* rpages[MAP_W][MAP_W];
} world_t;

void page_coords(int mm_x, int mm_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y);
void unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y);
void mm_from_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y);
void page_coords_from_unit_coords(int unit_x, int unit_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y);


int map_lidars(world_t* w, int n_lidars, lidar_scan_t** lidar_list, int* da, int* dx, int* dy);
void map_next_with_larger_search_area();

void map_sonar(world_t* w, sonar_scan_t* p_son);
void map_collision_obstacle(world_t* w, int32_t cur_ang, int cur_x, int cur_y, int stop_reason, int vect_valid, float vect_ang_rad);

int map_3dtof(world_t* w, int n_tofs, tof3d_scan_t** tof_list, int32_t *mx, int32_t *my);


void start_automapping_from_compass();
void start_automapping_skip_compass();
void stop_automapping();
void start_automap_only_compass();
void autofsm();
void dbg_test();
void clear_within_robot(world_t* w, pos_t pos);


#endif
