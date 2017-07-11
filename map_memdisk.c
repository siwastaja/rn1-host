#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "mapping.h"
#include "map_memdisk.h"

extern uint32_t robot_id;

int write_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];

	if(snprintf(fname, 1024, MAP_DIR"/%08x_%u_%u_%u.map", robot_id, w->id, pagex, pagey) > 1022)
		fname[1023] = 0;

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
	w->changed[pagex][pagey] = 0;

	return 0;
}

int read_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];
	if(snprintf(fname, 1024, MAP_DIR"/%08x_%u_%u_%u.map", robot_id, w->id, pagex, pagey) > 1022)
		fname[1023] = 0;

	//printf("Info: Attempting to read map page %s\n", fname);

	w->changed[pagex][pagey] = 0;

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
//		printf("Info: Allocating mem for page %d,%d\n", pagex, pagey);
		w->pages[pagex][pagey] = calloc(1, sizeof(map_page_t));
	}

	int ret = read_map_page(w, pagex, pagey);
	if(ret == 2)
	{
//		printf("Info: map page file didn't exist, initializing empty map page\n");
//		memset(w->pages[pagex][pagey], 0, sizeof(map_page_t));
	}
	else if(ret)
	{
		printf("Error: Reading map page file failed. Initializing empty map page\n");
//		memset(w->pages[pagex][pagey], 0, sizeof(map_page_t));
		return 1;
	}
	return 0;
}

int unload_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
		if(w->changed[pagex][pagey])
		{
			if(write_map_page(w, pagex, pagey))
			{
				printf("Error: writing map page (%d,%d) to disk failed\n", pagex, pagey);
			}
		}
//		printf("Info: Freeing mem for page %d,%d\n", pagex, pagey);
		free(w->pages[pagex][pagey]);
		w->pages[pagex][pagey] = 0;
		w->changed[pagex][pagey] = 0;

		free(w->rpages[pagex][pagey]);
		w->rpages[pagex][pagey] = 0;
	}
	else
	{
		printf("Warn: Trying to unload a map page which is already free.\n");
	}
	return 0;
}

int unload_map_pages(world_t* w, int cur_pagex, int cur_pagey)
{
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(w->pages[x][y] && (abs(cur_pagex - x) > 2 || abs(cur_pagey - y) > 2))
			{
				unload_map_page(w, x, y);
			}

		}
	}
	return 0;
}

int save_map_pages(world_t* w)  // returns number of changed pages
{
	int ret = 0;
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(w->pages[x][y] && w->changed[x][y])
			{
				ret++;
				write_map_page(w, x, y);
			}
		}
	}
	return ret;
}


void load_9pages(world_t* w, int pagex, int pagey)
{
	for(int x=-1; x<2; x++)
	{
		for(int y=-1; y<2; y++)
		{
			int xx = pagex+x;
			int yy = pagey+y;
			if(!w->pages[xx][yy])
			{
				load_map_page(w, xx, yy);
			}
		}
	}
}

void load_1page(world_t* w, int pagex, int pagey)
{
	if(!w->pages[pagex][pagey])
	{
		load_map_page(w, pagex, pagey);
	}
}

