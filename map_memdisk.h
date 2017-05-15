#ifndef MAP_MEMDISK_H
#define MAP_MEMDISK_H

#include <stdint.h>
#include "mapping.h"

// Disk access; file name is generated and the page is stored/read.
int write_map_page(world_t* w, int pagex, int pagey);
int read_map_page(world_t* w, int pagex, int pagey);

// Allocates memory for a page and reads page from disk; if it doesn't exist, the new page is zeroed out
int load_map_page(world_t* w, int pagex, int pagey);

// Writes the map page to disk and frees the memory, setting the page pointer to 0.
int unload_map_page(world_t* w, int pagex, int pagey);


// Loads requested pagex, pagey and 8 pages around it.
void load_9pages(world_t* w, int pagex, int pagey);

// Scans through the world and unloads any loaded pages farther than 2 pages away from cur_pagex, cur_pagey.
int unload_map_pages(world_t* w, cur_pagex, cur_pagey);


#endif
