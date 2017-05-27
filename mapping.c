#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323
#endif

#include "mapping.h"

world_t world;

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


/* Rotation:
x2 = x*cos(a) + y*sin(a)
y2 = -1*x*sin(a) + y*cos(a)
*/


static int score(world_t* w, int n_lidars, lidar_scan_t** lidar_list, 
	         int32_t da, int32_t dx, int32_t dy,
	         int* n_matched_walls, int* n_exactly_matched_walls, int* n_new_walls)
{
	int pagex, pagey, offsx, offsy;

	int n_points = 0;
	int n_matches = 0;

	// Go through all valid points in all lidars in the lidar_list.
	for(int l=0; l<n_lidars; l++)
	{
		lidar_scan_t* lid = lidar_list[l];

		// 9 pages will cover the full lidar scan by design.
		page_coords(lid->robot_pos.x, lid->robot_pos.y, &pagex, &pagey, &offsx, &offsy);
		load_9pages(&world, pagex, pagey);
		

		for(int p=0; p<LIDAR_SCAN_POINTS; p++)
		{
			if(!lid->scan[p].valid)
				continue;

			n_points++;

			// Rotate the point by da and then shift by dx, dy.
			float ang = (float)da/((float)ANG_1_DEG*360.0)*2.0*M_PI;
			int x = lid->scan[p].x*cos(ang) + lid->scan[p].y*sin(ang) + dx;
			int y = -1*lid->scan[p].x*sin(ang) + lid->scan[p].y*cos(ang) + dy;

			int is_match = 0;
			int is_exact = 0;
			int is_new_wall = 1;

			// Wall in any neighbouring cell is considered a match.
			// Not a wall in all neighbouring cells is considered a new wall (in old spot with no prior wall)
			// Else, we just have a wall in previously unmapped area.
			for(int ix=-1; ix<=1; ix++)
			{
				for(int iy=-1; iy<=1; iy++)
				{
					page_coords(x, y, &pagex, &pagey, &offsx, &offsy);
					if(w->pages[pagex][pagey]->units[offsx][offsy].result & UNIT_WALL)
					{
						is_match = 1;
						if(ix==0 && iy==0) 
							is_exact = 1;
					}
					else
					{
						if(w->pages[pagex][pagey]->units[offsx][offsy].num_seen)

					}

				}
			}

						n_matches++;
						n_exacts++;
			
		}
	}

	// Write the results
	if(n_matched_walls) *n_matched_walls = n_matches;
	if(n_exactly_matched_walls) *n_exactly_matched_walls = n_exacts;
	if(n_new_walls) *n_new_walls = n_points - n_matches;

	// Return the score: bigger = better
	// Exact matches have a slight effect on the result.
	return n_matches*10 + n_exacts;
}
	

/*
map_lidars takes a set of lidar scans, assumes they are in sync (i.e., robot coordinates relative
between the images are correct enough, searches for the map around expected coordinates to find
the location where the scans fit the best. Last, the map is modified with the new data.

This function is supposed to be used with a set of lidar scans that complement each other's blind
areas, and to some extent, expand the lidar range (in moderation). For example, the combined image
from 10 scans, that ranges a 5 x 8 meter area with 2000 points, instead of a single 5x5 meter image with 200 points,
would do great!

With single images, mapping is a bit uncertain, because if the image has rather few points, it has high
chances of accidentally matching another similar-looking area; even within the smallish search range.

On the other hand, using too many images covering a large area, or images that have high risk of being out of sync,
will result in a seriously messed up map, since the scans are assumed to be "in sync", and only
minor (one 40mm unit) adjustments will be made within the combined image.

For the same reason, you should use some mechanisms to choose images that are going to be in sync.

Note that this function also assumes that the scans have fairly well matching coordinates already, so this
does not implement "I'm lost, where am I?" functionality.

*/

int map_lidars(world_t* w, int n_lidars, lidar_scan_t** lidar_list)
{

	printf("Info: Attempting to map %d lidar images\n", n_lidars);

	int best_score = 0;
	int best_da=0, best_dx=0; best_dy=0;
	for(int da=-10*ANG_1_DEG; da<=10*ANG_1_DEG; da+=ANG_1_DEG)
	{
		for(int dx=-400; dx<=400; dx+=80)
		{
			for(int dy=-400; dy<=400; dy+=80)
			{
				int score_now = score(w, n_lidars, lidar_list, 
					da, dx, dy, 0, 0, 0);

				if(score_now > best_score)
				{
					best_score = score_now;
					best_da = da;
					best_dx = dx;
					best_dy = dy;
				}
			}
		}
	}

}

