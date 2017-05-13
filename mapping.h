#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>


#define UNIT_FREE	0
#define UNIT_ITEM	(1<<0)	// Small obstacle, detected by sonars or bumping into it
#define UNIT_WALL	(1<<1)	// Obstacle seen by the lidar.


/*
Map unit is a 40mm*40mm area. Memory usage is carefully considered, because the world is a 2D map of the map units.
40mm*40mm was selected as a good compromise between accuracy (i.e., considering that the 500mm wide robot can maneuver through
tight passages, which would be wasteful if the unit was larger) and memory/disk usage.
*/

typedef struct __attribute__ ((packed))
{
	uint16_t timestamp;	// Latest time mapped

	uint8_t latest;  	// Latest mapping result
	uint8_t num_seen;  	// Number of times mapped. Saturated at 255.

	uint8_t num_obstacles;  // "is an obstacle" counter. Every time mapped, ++ if obstacle, -- if not. Saturated at 255.
	uint8_t reserved;

	uint16_t reserved;
	
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

typedef struct
{
	map_unit_t units[MAP_PAGE_W][MAP_PAGE_W];
} map_page_t;



/*
world_t is one continuously mappable entity. There can be several worlds, but the worlds cannot overlap;
in case they would, they should be combined.

When the software cannot decide which world it's in, we create a new, empty world. If we figure out it matches
an earlier world, we will combine them.
*/
typedef struct
{
	uint32_t id;
} world_t;



#endif
