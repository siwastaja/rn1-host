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

int write_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];

	sprintf(fname, "%08x_%u_%u_%u.map", robot_id, w->id, pagex, pagey);

	printf("Info: writing map page %s\n", fname);

	FILE *f = fopen(fname, "w");
	if(!f)
	{
		fprintf(stderr, "Error %d opening %s for write\n", errno, fname);
		return 1;
	}

	if(fwrite(w->pages[pagex][pagey], sizeof(map_page_t), 1, f) != 1)
	{
		printf("Error: Writing map data failed\n");
	}
	fclose(f);
	return 0;
}

int read_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];
	sprintf(fname, "%08x_%u_%u_%u.map", robot_id, w->id, pagex, pagey);

	printf("Info: Attempting to read map page %s\n", fname);

	FILE *f = fopen(fname, "r");
	if(!f)
	{
		if(errno == ENOENT)
			return 2;
		fprintf(stderr, "Error %d opening %s for read\n", errno, fname);
		return 1;
	}

	if(fread(w->pages[pagex][pagey], sizeof(map_page_t), 1, f) != 1)
	{
		printf("Error: Reading map data failed\n");
	}

	fclose(f);
	return 0;
}

int load_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
		printf("Info: reloading already allocated map page %d,%d\n", pagex, pagey);
	}
	else
	{
		printf("Info: Allocating mem for page %d,%d\n", pagex, pagey);
		w->pages[pagex][pagey] = malloc(sizeof(map_page_t));
	}

	int ret = read_map_page(w, pagex, pagey);
	if(ret == 2)
	{
		printf("Info: map file didn't exist, initializing empty map\n");
		memset(w->pages[pagex][pagey], 0, sizeof(map_page_t));
	}
	else if(ret)
	{
		printf("Error: Reading map file failed. Initializing empty map\n");
		memset(w->pages[pagex][pagey], 0, sizeof(map_page_t));
	}
}

int unload_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
		if(write_map_page(w, pagex, pagey))
		{
			printf("Error: writing map page (%d,%d) to disk failed\n", pagex, pagey);
		}
		printf("Info: Freeing mem for page %d,%d\n", pagex, pagey);
		free(w->pages[pagex][pagey]);
		w->pages[pagex][pagey] = 0;
	}
	else
	{
		printf("Warn: Trying to unload a map page which is already free.\n");
	}
}

int unload_map_pages(world_t* w)
{
	int idx_x = 0, idx_y = 0, offs_x = 0, offs_y = 0;

	page_coords(cur_x, cur_y, &idx_x, &idx_y, &offs_x, &offs_y);

	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(w->pages[x][y] && (abs(idx_x - x) > 2 || abs(idx_y - y) > 2))
			{
				unload_map_page(w, x, y);
			}

		}
	}
}

void load_9pages(world_t* w, int pagex, int pagey)
{
	for(int x=-1; x<2; x++)
	{
		for(int y=-1; y<2; y++)
		{
			int xx = pagex+x;
			int yy = pagey+y;
			if(!world.pages[xx][yy])
			{
				load_map_page(&world, xx, yy);
			}
		}
	}
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



