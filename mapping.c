#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "mapping.h"

uint32_t robot_id = 0xacdcabba; // Hopefully unique identifier for the robot.

world_t world;

int cur_x, cur_y; // in mm

void page_coords(int mm_x, int mm_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int unit_x = mm_x / MAP_UNIT_W;
	int unit_y = mm_y / MAP_UNIT_W;
	unit_x += MAP_MIDDLE_UNIT;
	unit_y += MAP_MIDDLE_UNIT;
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}


int main(int argc, char** argv)
{

	int mm_x = 0;
	int mm_y = 0;
	while(1)
	{

		int idx_x, idx_y, offs_x, offs_y;

		page_coords(mm_x, mm_y, &idx_x, &idx_y, &offs_x, &offs_y);

		printf("Page X %d  Page Y %d  Offs X %d  Offs Y %d\n\n", idx_x, idx_y, offs_x, offs_y);

		load_9pages(&world, idx_x, idx_y);
		
		unload_map_pages(&world);

		printf("x?\n");
		scanf("%d", &mm_x);
		printf("y?\n");
		scanf("%d", &mm_y);

	}


	return 0;
}



