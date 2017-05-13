#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

uint32_t robot_id; // Hopefully unique identifier for the robot.

int cur_x, cur_y; // in mm

map_page_t** mem_pages;

int write_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];

	sprintf(fname, "%08x_%u_%d_%d.map", robot_id, w->id, pagex, pagey);
	FILE *f = fopen(fname, "w");
	if(!f)
	{
		fprintf(stderr, "Error %d opening %s for write\n", errno, fname);
		return 1;
	}
	return 0;
}

int read_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];

	sprintf(fname, "%08x_%u_%d_%d.map", robot_id, w->id, pagex, pagey);
	FILE *f = fopen(fname, "r");
	if(!f)
	{
		if(errno == ENOENT)
			return 2;
		fprintf(stderr, "Error %d opening %s for read\n", errno, fname);
		return 1;
	}
	return 0;
}

int main(int argc, char** argv)
{



	free(mem_pages);
	return 0;
}



