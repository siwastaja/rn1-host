#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323
#endif

#include "datatypes.h"
#include "map_memdisk.h"
#include "mapping.h"
#include "hwdata.h"
#include "routing.h"

#include "tcp_comm.h"   // to send dbgpoint.
#include "tcp_parser.h" // to send dbgpoint.

static const int search_order[25][2] = { 
	{ 0, 0},
	{ 0, 1},
	{ 0,-1},
	{ 1, 0},
	{-1, 0},
	{ 1, 1},
	{ 1,-1},
	{-1, 1},
	{-1,-1},
	{ 0, 2},
	{ 0,-2},
	{ 1, 2},
	{ 1,-2},
	{-1, 2},
	{-1,-2},
	{ 2, 0},
	{-2, 0},
	{ 2, 1},
	{ 2,-1},
	{-2, 1},
	{-2,-1},
	{ 2, 2},
	{ 2,-2},
	{-2, 2},
	{-2,-2}};


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

void unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y)
{
	int unit_x_t = mm_x / MAP_UNIT_W;
	int unit_y_t = mm_y / MAP_UNIT_W;
	unit_x_t += MAP_MIDDLE_UNIT;
	unit_y_t += MAP_MIDDLE_UNIT;

	*unit_x = unit_x_t;
	*unit_y = unit_y_t;
}

void mm_from_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y)
{
	unit_x -= MAP_MIDDLE_UNIT;
	unit_y -= MAP_MIDDLE_UNIT;

	*mm_x = unit_x * MAP_UNIT_W;
	*mm_y = unit_y * MAP_UNIT_W;
}

void page_coords_from_unit_coords(int unit_x, int unit_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}


// Shift page,offset coords directly by shift_x, shift_y units.
void shift_coords(int* px, int* py, int* ox, int* oy, int shift_x, int shift_y)
{
	*ox += shift_x;
	while(*ox > MAP_PAGE_W)
	{
		*ox -= MAP_PAGE_W;
		(*px)--;
	}
	while(*ox < 0)
	{
		*ox += MAP_PAGE_W;
		(*px)++;
	}

	*oy += shift_x;
	while(*oy > MAP_PAGE_W)
	{
		*oy -= MAP_PAGE_W;
		(*py)--;
	}
	while(*oy < 0)
	{
		*oy += MAP_PAGE_W;
		(*py)++;
	}
}


#define sq(x) ((x)*(x))

/*
	Go through every point in every lidar scan.
	Find closest point from every other scan. If far away,
	remove the point as being a moving object (or some kind of error).

	modifies lidar scans pointed by lidar_list.
*/

static int prefilter_lidar_list(int n_lidars, lidar_scan_t** lidar_list)
{
	int n_removed_per_scan[32] = {0};
	int n_removed = 0;

	for(int la=0; la<n_lidars; la++)
	{
		lidar_scan_t* lida = lidar_list[la];

		for(int pa=0; pa<LIDAR_SCAN_POINTS; pa++)
		{
			if(!lida->scan[pa].valid)
				continue;

			for(int lb=0; lb<n_lidars; lb++)
			{
				if(la == lb) continue;

				lidar_scan_t* lidb = lidar_list[lb];

				for(int pb=0; pb<LIDAR_SCAN_POINTS; pb++)
				{
					if(!lidb->scan[pb].valid)
						continue;

					int64_t dx = lidb->scan[pb].x - lida->scan[pa].x;
					int64_t dy = lidb->scan[pb].y - lida->scan[pa].y;
					int64_t dist = sq(dx) + sq(dy);

					if(dist < sq(100)) goto FOUND_NEAR;
				}
			}

			lida->scan[pa].valid = 0;
			n_removed_per_scan[la]++;
			n_removed++;

			FOUND_NEAR:;
		}
	}

	printf("INFO: prefilter_lidar_list() removed %d points: ", n_removed);
	for(int i=0; i < n_lidars; i++)	printf("%d, ", n_removed_per_scan[i]);
	printf("\n");
	return n_removed;
}


static int prefilter_lidar_list_aggressive(int n_lidars, lidar_scan_t** lidar_list)
{
	int n_removed_per_scan[32] = {0};
	int n_removed = 0;

	for(int la=0; la<n_lidars; la++)
	{
		lidar_scan_t* lida = lidar_list[la];

		for(int pa=0; pa<LIDAR_SCAN_POINTS; pa++)
		{
			if(!lida->scan[pa].valid)
				continue;

			int nears = 0;
			for(int lb=0; lb<n_lidars; lb++)
			{
				if(la == lb) continue;

				lidar_scan_t* lidb = lidar_list[lb];

				for(int pb=0; pb<LIDAR_SCAN_POINTS; pb++)
				{
					if(!lidb->scan[pb].valid)
						continue;

					int64_t dx = lidb->scan[pb].x - lida->scan[pa].x;
					int64_t dy = lidb->scan[pb].y - lida->scan[pa].y;
					int64_t dist = sq(dx) + sq(dy);

					if(dist < sq(80)) nears++;
				}
			}

			if(nears < n_lidars/2)
			{
				lida->scan[pa].valid = 0;
				n_removed_per_scan[la]++;
				n_removed++;
			}
		}
	}

	printf("INFO: prefilter_lidar_list_aggressive() removed %d points: ", n_removed);
	for(int i=0; i < n_lidars; i++)	printf("%d, ", n_removed_per_scan[i]);
	printf("\n");
	return n_removed;
}


/* Rotation:
x2 = x*cos(a) + y*sin(a)
y2 = -1*x*sin(a) + y*cos(a)
*/

static int score(world_t* w, int n_lidars, lidar_scan_t** lidar_list, 
	         int32_t da, int32_t dx, int32_t dy, int32_t rotate_mid_x, int32_t rotate_mid_y,
	         int* n_matched_walls, int* n_exactly_matched_walls, int* n_new_walls, int* n_discovered_walls)
{
	int pagex, pagey, offsx, offsy;

	int n_points = 0;
	int n_matches = 0;
	int n_news = 0;
	int n_exacts = 0;
	int n_steadys = 0;

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
	
			int pre_x = lid->scan[p].x - rotate_mid_x;
			int pre_y = lid->scan[p].y - rotate_mid_y;

			int x = pre_x*cos(ang) + pre_y*sin(ang) + rotate_mid_x + dx;
			int y = -1*pre_x*sin(ang) + pre_y*cos(ang) + rotate_mid_y + dy;

			int is_match = 0;
			int seen_with_no_wall = 0;
			int is_exact = 0;
			int is_steady = 0; // There is a wall in the decided result field.

			// Wall in any neighbouring cell is considered a match.

			page_coords(x, y, &pagex, &pagey, &offsx, &offsy);
			if(w->pages[pagex][pagey]->units[offsx][offsy].result & UNIT_WALL)
			{
				is_match = 1;
				is_exact = 1;
				is_steady = 1;
			}
			else if(w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles)
			{
				is_match = 1;
				is_exact = 1;
			}
			else
			{
				for(int ix=-1*MAP_UNIT_W; ix<=MAP_UNIT_W; ix+=MAP_UNIT_W)
				{
					for(int iy=-1*MAP_UNIT_W; iy<=MAP_UNIT_W; iy+=MAP_UNIT_W)
					{
						page_coords(x+ix, y+iy, &pagex, &pagey, &offsx, &offsy);
						if(w->pages[pagex][pagey]->units[offsx][offsy].result & UNIT_WALL)
						{
							is_match = 1;
							is_steady = 1;
							break;
						}
						else if(w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles)
						{
							is_match = 1;
						}
						else if(w->pages[pagex][pagey]->units[offsx][offsy].result & UNIT_MAPPED)
						{
							seen_with_no_wall++;
						}
					}
				}
			}

			if(is_match) n_matches++;
			if(is_exact) n_exacts++;
			if(is_steady) n_steadys++;
			// There is no wall, and most of the 9 units were mapped "no wall" before, so we have a new wall:
			if(!is_match && seen_with_no_wall > 6) n_news++;

			
			
		}
	}

	// Write the results
	if(n_matched_walls) *n_matched_walls = n_matches;
	if(n_exactly_matched_walls) *n_exactly_matched_walls = n_exacts;
	if(n_new_walls) *n_new_walls = n_news;
	if(n_discovered_walls) *n_discovered_walls = n_points - n_matches;

	// Return the score: bigger = better
	// Exact matches have a slight effect on the result.
	// New walls decrease the score.
	return n_matches*5 + n_steadys*3 + n_exacts*1 - n_news*5;
}

#define TEMP_MAP_W (3*MAP_PAGE_W)
#define TEMP_MAP_MIDDLE (TEMP_MAP_W/2)

// Slower, allows stepping larger steps:
static int gen_scoremap_for_large_steps(world_t *w, int8_t *scoremap, int mid_x, int mid_y)
{
	int px, py, ox, oy;

	printf("Generating scoremap (for large steps)..."); fflush(stdout);
	for(int xx = 0; xx < TEMP_MAP_W; xx++)
	{
		for(int yy = 0; yy < TEMP_MAP_W; yy++)
		{
			page_coords(mid_x + (xx-TEMP_MAP_MIDDLE)*MAP_UNIT_W, mid_y + (yy-TEMP_MAP_MIDDLE)*MAP_UNIT_W, &px, &py, &ox, &oy);
			load_9pages(w, px, py);

			int score = 8*w->pages[px][py]->units[ox][oy].num_obstacles - 4*w->pages[px][py]->units[ox][oy].num_seen;

			for(int ix=-2; ix<=2; ix++)
			{
				for(int iy=-2; iy<=2; iy++)
				{
					int npx = px, npy = py, nox = ox + ix, noy = oy + iy;
					if(nox < 0) { nox += MAP_PAGE_W; npx--; } else if(nox >= MAP_PAGE_W) { nox -= MAP_PAGE_W; npx++;}
					if(noy < 0) { noy += MAP_PAGE_W; npy--; } else if(noy >= MAP_PAGE_W) { noy -= MAP_PAGE_W; npy++;}

					int neigh_score;
					if(ix == -2 || ix == 2 || iy == -2 || iy == 2)
						neigh_score = 5*w->pages[npx][npy]->units[nox][noy].num_obstacles - 4*w->pages[npx][npy]->units[nox][noy].num_seen;
					else
						neigh_score = 6*w->pages[npx][npy]->units[nox][noy].num_obstacles - 4*w->pages[npx][npy]->units[nox][noy].num_seen;
					if(neigh_score > score) score = neigh_score;
				}
			}

			score >>= 1;

			if(score > 63) score=63; else if(score < -64) score = -64;

			scoremap[yy*TEMP_MAP_W+xx] = score;
		}
	}

/*
	// Output 768x768x24bit raw image for debug.
	FILE* dbg_f = fopen("dbg_scoremap.data", "w");

	for(int iy = 0; iy < TEMP_MAP_W; iy++)
	{
		for(int ix = 0; ix < TEMP_MAP_W; ix++)
		{
			int r = 0, g = 0;
			if(scoremap[iy*TEMP_MAP_W+ix] > 0)
				g = scoremap[iy*TEMP_MAP_W+ix]*4;
			else
				r = scoremap[iy*TEMP_MAP_W+ix]*-4;

			if(r > 255) r = 255; if(g > 255) g = 255;
			fputc(r, dbg_f); // R
			fputc(g, dbg_f); // G
			fputc(0, dbg_f); // B
		}
	}

	fclose(dbg_f);
*/
	printf(" OK.\n");


	return 0;
}


static int gen_scoremap_for_small_steps(world_t *w, int8_t *scoremap, int mid_x, int mid_y)
{
	int px, py, ox, oy;

	printf("Generating scoremap..."); fflush(stdout);
	for(int xx = 0; xx < TEMP_MAP_W; xx++)
	{
		for(int yy = 0; yy < TEMP_MAP_W; yy++)
		{
			page_coords(mid_x + (xx-TEMP_MAP_MIDDLE)*MAP_UNIT_W, mid_y + (yy-TEMP_MAP_MIDDLE)*MAP_UNIT_W, &px, &py, &ox, &oy);
			load_9pages(w, px, py);

			int score = 4*w->pages[px][py]->units[ox][oy].num_obstacles; // - 2*w->pages[px][py]->units[ox][oy].num_seen;

			for(int ix=-1; ix<=1; ix++)
			{
				for(int iy=-1; iy<=1; iy++)
				{
					int npx = px, npy = py, nox = ox + ix, noy = oy + iy;
					if(nox < 0) { nox += MAP_PAGE_W; npx--; } else if(nox >= MAP_PAGE_W) { nox -= MAP_PAGE_W; npx++;}
					if(noy < 0) { noy += MAP_PAGE_W; npy--; } else if(noy >= MAP_PAGE_W) { noy -= MAP_PAGE_W; npy++;}

					int neigh_score = 3*w->pages[npx][npy]->units[nox][noy].num_obstacles; // - 2*w->pages[npx][npy]->units[nox][noy].num_seen;
					if(neigh_score > score) score = neigh_score;
				}
			}

			score >>= 1;
			if(score > 63) score=63; else if(score < -64) score = -64;

			scoremap[yy*TEMP_MAP_W+xx] = score;
		}
	}

/*
	// Output 768x768x24bit raw image for debug.
	FILE* dbg_f = fopen("dbg_scoremap.data", "w");

	for(int iy = 0; iy < TEMP_MAP_W; iy++)
	{
		for(int ix = 0; ix < TEMP_MAP_W; ix++)
		{
			int r = 0, g = 0;
			if(scoremap[iy*TEMP_MAP_W+ix] > 0)
				g = scoremap[iy*TEMP_MAP_W+ix]*4;
			else
				r = scoremap[iy*TEMP_MAP_W+ix]*-4;

			if(r > 255) r = 255; if(g > 255) g = 255;
			fputc(r, dbg_f); // R
			fputc(g, dbg_f); // G
			fputc(0, dbg_f); // B
		}
	}

	fclose(dbg_f);
*/
	printf(" OK.\n");


	return 0;
}


/*
	score_quick uses 3*MAP_PAGE_W*3*MAP_PAGE_W*int8_t scoremap, with middlepoint at rotate_mid_x, rotate_mix_y.
*/

static int32_t score_quick(int8_t *scoremap, int n_lidars, lidar_scan_t** lidar_list, 
	         int32_t da, int32_t dx, int32_t dy, int32_t rotate_mid_x, int32_t rotate_mid_y)
{
	int n_points = 0;
	int score = 0;

	// Go through all valid points in all lidars in the lidar_list.
	for(int l=0; l<n_lidars; l++)
	{
		lidar_scan_t* lid = lidar_list[l];

		for(int p=0; p<LIDAR_SCAN_POINTS; p++)
		{
			if(!lid->scan[p].valid)
				continue;

			n_points++;

			// Rotate the point by da and then shift by dx, dy.
			float ang = (float)da/((float)ANG_1_DEG*360.0)*2.0*M_PI;

			int pre_x = lid->scan[p].x - rotate_mid_x;
			int pre_y = lid->scan[p].y - rotate_mid_y;

			int x = pre_x*cos(ang) + pre_y*sin(ang) + dx;
			int y = -1*pre_x*sin(ang) + pre_y*cos(ang) + dy;

			x /= MAP_UNIT_W; y /= MAP_UNIT_W;
			x += TEMP_MAP_MIDDLE; y += TEMP_MAP_MIDDLE;

/*			if(x < 1 || x >= 767 || y < 1 || y >= 767)
			{
				printf("Error: illegal indexes in score_quick: (%d, %d)\n", x, y);
				return -99999;
			}
*/
			score += scoremap[y*TEMP_MAP_W+x];	
		}
	}

	return (1000*score)/n_points;
}

static int32_t score_quick_search_xy(int8_t *scoremap, int n_lidars, lidar_scan_t** lidar_list, 
	       int32_t rotate_mid_x, int32_t rotate_mid_y, 
	       int32_t da, int32_t dx_start, int32_t dx_step, int32_t num_dx, int32_t dy_start, int32_t dy_step, int32_t num_dy,
               int32_t *best_dx, int32_t *best_dy)
{
	int n_points = 0;

//	printf("score_quick_search_xy: dx: %d, %d, %d   dy: %d, %d, %d\n", dx_start, dx_step, num_dx, dy_start, dy_step, num_dy);
	if(num_dx > 32 || num_dy > 32 || num_dx < 1 || num_dy < 1)
	{
		printf("ERROR: score_quick_search_xy(): Invalid num_dx or num_dy\n.");
		exit(1);
	}

	int score[32][32] = {{0}};

	float ang = (float)da/((float)ANG_1_DEG*360.0)*2.0*M_PI;

	// Go through all valid points in all lidars in the lidar_list.
	for(int l=0; l<n_lidars; l++)
	{
		lidar_scan_t* lid = lidar_list[l];

		for(int p=0; p<LIDAR_SCAN_POINTS; p++)
		{
			if(!lid->scan[p].valid)
				continue;

			n_points++;

			// Rotate the point by da and then shift by dx, dy.

			int pre_x = lid->scan[p].x - rotate_mid_x;
			int pre_y = lid->scan[p].y - rotate_mid_y;

			int rotated_x = pre_x*cos(ang) + pre_y*sin(ang);
			int rotated_y = -1*pre_x*sin(ang) + pre_y*cos(ang);

			for(int ix = 0; ix < num_dx; ix++)
			{
				for(int iy = 0; iy < num_dy; iy++)
				{
					int x = rotated_x + dx_start+dx_step*ix;
					int y = rotated_y + dy_start+dy_step*iy;

					x /= MAP_UNIT_W; y /= MAP_UNIT_W;
					x += TEMP_MAP_MIDDLE; y += TEMP_MAP_MIDDLE;

			/*			if(x < 1 || x >= 767 || y < 1 || y >= 767)
					{
						printf("Error: illegal indexes in score_quick: (%d, %d)\n", x, y);
						return -99999;
					}
			*/
					score[ix][iy] += scoremap[y*TEMP_MAP_W+x];
				}
			}
		}
	}

//	printf("scores: ");
	int best_score = -999999, best_ix = 0, best_iy = 0;
	for(int ix = 0; ix < num_dx; ix++)
	{
		for(int iy = 0; iy < num_dy; iy++)
		{
//			printf(" (%2d,%2d): %6d ", ix, iy, score[ix][iy]);
			if(score[ix][iy] > best_score)
			{
				best_score = score[ix][iy];
				best_ix = ix;
				best_iy = iy;
			}
		}
	}

//	printf("\n");
	*best_dx = dx_start+dx_step*best_ix;
	*best_dy = dy_start+dy_step*best_iy;

	return (1000*best_score)/n_points;
}


typedef struct  // Each bit represents each lidar scan (i.e., 32 lidar scans max).
{
	uint32_t seen;
	uint32_t wall;
} temp_map_img_t;

static int do_mapping(world_t* w, int n_lidars, lidar_scan_t** lidar_list,
                      int32_t da, int32_t dx, int32_t dy, int32_t rotate_mid_x, int32_t rotate_mid_y,
                      int32_t *after_dx, int32_t *after_dy)
{
	int pagex, pagey, offsx, offsy;

	*after_dx = 0;
	*after_dy = 0;

	/*
		Generate temporary map, counting seen areas / wall areas.
		This map is (3* MAP_PAGE_W) x (3* MAP_PAGE_W) in size, the middle point being at rotate_mid_x,rotate_mid_y.
	*/

	temp_map_img_t* temp_map = calloc(3*MAP_PAGE_W*3*MAP_PAGE_W, sizeof(temp_map_img_t));
	if(!temp_map)
	{
		printf("ERROR: Out of memory in do_mapping\n");
		return -1;
	}

	// Go through all valid points in all lidars in the lidar_list.
	for(int l=0; l<n_lidars; l++)
	{
		lidar_scan_t* lid = lidar_list[l];

		// Rotate the point by da and then shift by dx, dy.
		float ang = (float)da/((float)ANG_1_DEG*360.0)*2.0*M_PI;

		int robot_pre_x = lid->robot_pos.x - rotate_mid_x;
		int robot_pre_y = lid->robot_pos.y - rotate_mid_y;

		int robot_x = robot_pre_x*cos(ang) + robot_pre_y*sin(ang) /* + rotate_mid_x */ + dx ;
		int robot_y = -1*robot_pre_x*sin(ang) + robot_pre_y*cos(ang) /* + rotate_mid_y */ + dy;

		robot_x /= MAP_UNIT_W; robot_y /= MAP_UNIT_W;
		robot_x += TEMP_MAP_MIDDLE; robot_y += TEMP_MAP_MIDDLE;

		// Robot coords should be in the middle 1/3..2/3 section.
		if(robot_x < MAP_PAGE_W || robot_x >= 2*MAP_PAGE_W || robot_y < MAP_PAGE_W || robot_y > 2*MAP_PAGE_W)
		{
			printf("ERROR: out of range temp map coords (%d, %d) (robot position)\n", robot_x, robot_y);
			free(temp_map);
			return -2;
		}
		
		for(int p=0; p<LIDAR_SCAN_POINTS; p++)
		{
			if(!lid->scan[p].valid)
				continue;

			// Rotate the point by da and then shift by dx, dy.	
			int pre_x = lid->scan[p].x - rotate_mid_x;
			int pre_y = lid->scan[p].y - rotate_mid_y;

			int x = pre_x*cos(ang) + pre_y*sin(ang) /* + rotate_mid_x */ + dx ;
			int y = -1*pre_x*sin(ang) + pre_y*cos(ang) /* + rotate_mid_y */ + dy;

			x /= MAP_UNIT_W; y /= MAP_UNIT_W;

			x += TEMP_MAP_MIDDLE; y += TEMP_MAP_MIDDLE;

			if(x < 0 || x >= 3*MAP_PAGE_W || y < 0 || y > 3*MAP_PAGE_W)
			{
				printf("ERROR: out of range temp map coords (%d, %d) (scan point)\n", x, y);
				free(temp_map);
				return -2;
			}

			// Mark areas between the robot coords and the current point: "seen".

			int dx = x - robot_x;
			int dy = y - robot_y;

			if(abs(dx) >= abs(dy)) // Step in X direction
			{
				if(dx >= 0)
				{
					float dy_per_step = (float)dy/(float)dx;
					for(int ix = 0; ix < dx; ix++)
					{
						int cur_y = robot_y + dy_per_step*(float)ix;
						int cur_x = robot_x + ix;
						temp_map[cur_y*TEMP_MAP_W + cur_x].seen |= 1UL<<l;
					}
				}
				else // dx < 0
				{
					float dy_per_step = (float)dy/(float)dx;
					for(int ix = 0; ix < -1*dx; ix++)
					{
						int cur_y = robot_y - dy_per_step*(float)ix;
						int cur_x = robot_x - ix;
						temp_map[cur_y*TEMP_MAP_W + cur_x].seen |= 1UL<<l;
					}
				}

			}
			else // Step in Y direction
			{
				if(dy >= 0)
				{
					float dx_per_step = (float)dx/(float)dy;
					for(int iy = 0; iy < dy; iy++)
					{
						int cur_x = robot_x + dx_per_step*(float)iy;
						int cur_y = robot_y + iy;
						temp_map[cur_y*TEMP_MAP_W + cur_x].seen |= 1UL<<l;
					}
				}
				else // dy < 0
				{
					float dx_per_step = (float)dx/(float)dy;
					for(int iy = 0; iy < -1*dy; iy++)
					{
						int cur_x = robot_x - dx_per_step*(float)iy;
						int cur_y = robot_y - iy;
						temp_map[cur_y*TEMP_MAP_W + cur_x].seen |= 1UL<<l;
					}
				}

			}

			// Finally, mark the lidar point as a wall, at the end of the "seen" vector
			temp_map[y*TEMP_MAP_W + x].wall |= 1UL<<l;
		}
	}

/*
	// Output 768x768x24bit raw image for debug.
	FILE* dbg_f = fopen("dbg_image_before.data", "w");

	for(int iy = 0; iy < TEMP_MAP_W; iy++)
	{
		for(int ix = 0; ix < TEMP_MAP_W; ix++)
		{
			int s_cnt = 0, w_cnt = 0;
			uint32_t tmp = temp_map[iy*TEMP_MAP_W+ix].seen;
			while(tmp)
			{
				s_cnt++;
				tmp>>=1;
			}
			tmp = temp_map[iy*TEMP_MAP_W+ix].wall;
			while(tmp)
			{
				w_cnt++;
				tmp>>=1;
			}

			s_cnt*=20;
			w_cnt*=20;

			fputc(w_cnt, dbg_f); // R
			fputc(s_cnt, dbg_f); // G
			fputc(0, dbg_f);
		}
	}

	fclose(dbg_f);

*/

	/*
		Processing round - try to remove duplicate wall units within the same vectors.
	*/

	int prev_visit_px = -1, prev_visit_py = -1, prev_visit_ox = 0, prev_visit_oy = 0;

	for(int l=0; l<n_lidars; l++)
	{
		lidar_scan_t* lid = lidar_list[l];

		// mark this as visited
		page_coords(lid->robot_pos.x, lid->robot_pos.y, &pagex, &pagey, &offsx, &offsy);
		if(pagex != prev_visit_px || pagey != prev_visit_py || offsx != prev_visit_ox || offsy != prev_visit_oy)
		{
			load_1page(w, pagex, pagey);
			PLUS_SAT_255(w->pages[pagex][pagey]->units[offsx][offsy].num_visited);
		}
		prev_visit_px = pagex; prev_visit_py = pagey; prev_visit_ox = offsx; prev_visit_oy = offsy;

		// Rotate the point by da and then shift by dx, dy.
		float ang = (float)da/((float)ANG_1_DEG*360.0)*2.0*M_PI;

		int robot_pre_x = lid->robot_pos.x - rotate_mid_x;
		int robot_pre_y = lid->robot_pos.y - rotate_mid_y;

		int robot_x = robot_pre_x*cos(ang) + robot_pre_y*sin(ang) /* + rotate_mid_x */ + dx ;
		int robot_y = -1*robot_pre_x*sin(ang) + robot_pre_y*cos(ang) /* + rotate_mid_y */ + dy;

		robot_x /= MAP_UNIT_W; robot_y /= MAP_UNIT_W;
		robot_x += TEMP_MAP_MIDDLE; robot_y += TEMP_MAP_MIDDLE;

		for(int p=0; p<LIDAR_SCAN_POINTS; p++)
		{
			if(!lid->scan[p].valid)
				continue;

			// Rotate the point by da and then shift by dx, dy.	
			int pre_x = lid->scan[p].x - rotate_mid_x;
			int pre_y = lid->scan[p].y - rotate_mid_y;

			int x = pre_x*cos(ang) + pre_y*sin(ang) /* + rotate_mid_x */ + dx ;
			int y = -1*pre_x*sin(ang) + pre_y*cos(ang) /* + rotate_mid_y */ + dy;

			x /= MAP_UNIT_W; y /= MAP_UNIT_W;

			x += TEMP_MAP_MIDDLE; y += TEMP_MAP_MIDDLE;

			// Find the next unit from where we did put the wall before.

			int dx = x - robot_x;
			int dy = y - robot_y;

			int next_x, next_y;
			if(abs(dx) >= abs(dy)) // Step in X direction
			{
				float dy_per_step = (float)dy/(float)dx;
				int next_dx = dx + ((dx>0)?1:-1);
				next_y = robot_y + dy_per_step*(float)next_dx;
				next_x = robot_x + next_dx;
			}
			else // Step in Y direction
			{
				float dx_per_step = (float)dx/(float)dy;
				int next_dy = dy + ((dy>0)?1:-1);
				next_x = robot_x + dx_per_step*(float)next_dy;
				next_y = robot_y + next_dy;

			}

			int w_cnt_at_next = 0;
			uint32_t tmp = temp_map[next_y*TEMP_MAP_W+next_x].wall;
			while(tmp) { w_cnt_at_next++; tmp>>=1;}

			int w_cnt_at_cur = 0;
			tmp = temp_map[y*TEMP_MAP_W+x].wall;
			while(tmp) { w_cnt_at_cur++; tmp>>=1;}

			if(w_cnt_at_next > 0 && w_cnt_at_cur > 0 && w_cnt_at_next > w_cnt_at_cur) // next spot wins
			{
				temp_map[next_y*TEMP_MAP_W+next_x].wall |= temp_map[y*TEMP_MAP_W + x].wall; // Mark all hits to the next spot.
				temp_map[y*TEMP_MAP_W + x].wall = 0; // remove the wall from where it was.
			}
			else if(w_cnt_at_cur > 0 && w_cnt_at_next > 0 && w_cnt_at_cur > w_cnt_at_next) // cur pos wins
			{
				temp_map[y*TEMP_MAP_W+x].wall |= temp_map[next_y*TEMP_MAP_W + next_x].wall; // Mark all those hits to the current spot
				temp_map[next_y*TEMP_MAP_W + next_x].wall = 0; // remove the wall from the next spot
			}
		}
	}
/*

	// Output 768x768x24bit raw image for debug.
	dbg_f = fopen("dbg_image_after.data", "w");

	for(int iy = 0; iy < TEMP_MAP_W; iy++)
	{
		for(int ix = 0; ix < TEMP_MAP_W; ix++)
		{
			int s_cnt = 0, w_cnt = 0;
			uint32_t tmp = temp_map[iy*TEMP_MAP_W+ix].seen;
			while(tmp)
			{
				s_cnt++;
				tmp>>=1;
			}
			tmp = temp_map[iy*TEMP_MAP_W+ix].wall;
			while(tmp)
			{
				w_cnt++;
				tmp>>=1;
			}

			s_cnt*=20;
			w_cnt*=20;

			fputc(w_cnt, dbg_f); // R
			fputc(s_cnt, dbg_f); // G
			fputc(0, dbg_f);
		}
	}

	fclose(dbg_f);
*/
	// Load relevant 9 pages in memory
	page_coords(rotate_mid_x, rotate_mid_y, &pagex, &pagey, &offsx, &offsy);
	load_9pages(&world, pagex, pagey);

	// Add our temporary map to the actual map.
	// Don't loop near to the edges, we are comparing neighbouring cells inside the loop.
	// Operate by reading a copy, writing to actual map, so that what we have just now written doesn't affect the adjacent units:

	
	int mid_x_mm = (rotate_mid_x/MAP_UNIT_W)*MAP_UNIT_W;
	int mid_y_mm = (rotate_mid_y/MAP_UNIT_W)*MAP_UNIT_W;
	page_coords(mid_x_mm, mid_y_mm, &pagex, &pagey, &offsx, &offsy);

	static map_page_t copies[3][3];
	static uint8_t spot_used[3][3][MAP_PAGE_W][MAP_PAGE_W];

	int copy_pagex_start = pagex-1;
	int copy_pagey_start = pagey-1;

	for(int i = 0; i<3; i++)
	{
		for(int o=0; o<3; o++)
		{
			memcpy(&copies[i][o], w->pages[copy_pagex_start+i][copy_pagey_start+o], sizeof(map_page_t));
			memset(spot_used[i][o], 0, MAP_PAGE_W*MAP_PAGE_W);
		}
	}

	int avg_drift_cnt = 0, avg_drift_x = 0, avg_drift_y = 0;

	for(int iy = 3; iy < TEMP_MAP_W-3; iy++)
	{
		for(int ix = 3; ix < TEMP_MAP_W-3; ix++)
		{
			int x_mm = (rotate_mid_x/MAP_UNIT_W - TEMP_MAP_MIDDLE + ix)*MAP_UNIT_W;
			int y_mm = (rotate_mid_y/MAP_UNIT_W - TEMP_MAP_MIDDLE + iy)*MAP_UNIT_W;
			page_coords(x_mm, y_mm, &pagex, &pagey, &offsx, &offsy);
//			if(ix == 2 && iy == 2)
//				printf("Info: temp map -> map: start: page (%d, %d) offs (%d, %d)\n", pagex, pagey, offsx, offsy);

//			float ang_from_middle = atan2(y_mm-rotate_mid_y, x_mm-rotete_mid_x)*(8.0/(2.0*M_PI));
//			if(ang_from_middle < 0.0) ang_from_middle += 8.0;
//			int ang_idx = ang_from_middle+0.5;

			int s_cnt = 0, w_cnt = 0, neigh_w_cnt = 0;
			uint32_t tmp = temp_map[iy*TEMP_MAP_W+ix].seen; while(tmp) { s_cnt++; tmp>>=1; }
			tmp = temp_map[iy*TEMP_MAP_W+ix].wall; while(tmp) { w_cnt++; tmp>>=1; }

			tmp = temp_map[(iy)*TEMP_MAP_W+(ix+1)].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy)*TEMP_MAP_W+(ix-1)].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy+1)*TEMP_MAP_W+(ix+1)].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy+1)*TEMP_MAP_W+(ix-1)].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy+1)*TEMP_MAP_W+(ix  )].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy-1)*TEMP_MAP_W+(ix+1)].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy-1)*TEMP_MAP_W+(ix-1)].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }
			tmp = temp_map[(iy-1)*TEMP_MAP_W+(ix  )].wall; while(tmp) { neigh_w_cnt++; tmp>>=1; }


			if(w_cnt > 3) // A wall is very clearly here.
			{
				int px = pagex, py = pagey;

				int copy_px = px - copy_pagex_start;
				int copy_py = py - copy_pagey_start;

				if(copy_px < 0 || copy_px > 2 || copy_py < 0 || copy_py > 2 || (copy_px == 0 && offsx < 3) || (copy_py == 0 && offsy < 3) ||
				   (copy_px == 2 && offsx > MAP_PAGE_W-4) || (copy_py == 2 && offsy > MAP_PAGE_W-4))
				{
					printf("ERROR: invalid copy_px (%d) and/or copy_py (%d)\n", copy_px, copy_py);
					free(temp_map);
					return -3;
				}

				int found = 0;
				for(int i=0; i<25; i++)
				{
					px = pagex;
					py = pagey;
					int ox = offsx+search_order[i][0];
					int oy = offsy+search_order[i][1];

					if(ox >= MAP_PAGE_W) {ox-=MAP_PAGE_W; px++;}
					else if(ox < 0) {ox+=MAP_PAGE_W; px--;}
					if(oy >= MAP_PAGE_W) {oy-=MAP_PAGE_W; py++;}
					else if(oy < 0) {oy+=MAP_PAGE_W; py--;}

					copy_px = px - copy_pagex_start;
					copy_py = py - copy_pagey_start;

					if((copies[copy_px][copy_py].units[ox][oy].num_obstacles))
					{
						if(!spot_used[copy_px][copy_py][ox][oy])
						{
							avg_drift_cnt++;
							avg_drift_x += search_order[i][0];
							avg_drift_y += search_order[i][1];

							// Existing wall here, it suffices, increase the seen count.
							PLUS_SAT_255(w->pages[px][py]->units[ox][oy].num_seen);
							PLUS_SAT_255(w->pages[px][py]->units[ox][oy].num_obstacles);

							if(w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles > 2)
								w->pages[pagex][pagey]->units[offsx][offsy].result |= UNIT_WALL;

							spot_used[copy_px][copy_py][ox][oy] = 1;
							w->changed[px][py] = 1;
							found = 1;
							break;
						}
					}
				}

				if(!found)
				{
					// We have a new wall.
					w->pages[pagex][pagey]->units[offsx][offsy].result |= UNIT_MAPPED;
					PLUS_SAT_255(w->pages[pagex][pagey]->units[offsx][offsy].num_seen);
					PLUS_SAT_255(w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles);
					w->changed[pagex][pagey] = 1;
				}
			}

			if(w_cnt == 0 && s_cnt > 3)
			{
				// We don't have a wall, but we mapped this unit nevertheless.
				w->pages[pagex][pagey]->units[offsx][offsy].result |= UNIT_MAPPED;
				PLUS_SAT_255(w->pages[pagex][pagey]->units[offsx][offsy].num_seen);

				if(!(w->pages[pagex][pagey]->units[offsx][offsy].result & UNIT_DO_NOT_REMOVE_BY_LIDAR))
				{
					MINUS_SAT_0(w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles);

					if(
					   ( s_cnt > 5 && neigh_w_cnt == 0 && // we are quite sure:
					   ((int)w->pages[pagex][pagey]->units[offsx][offsy].num_seen > (2*(int)w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles + 3)))
					   || (neigh_w_cnt < 2 &&  // there is 1 wall neighbor, so we are not so sure, but do it eventually.
					   ((int)w->pages[pagex][pagey]->units[offsx][offsy].num_seen > (5*(int)w->pages[pagex][pagey]->units[offsx][offsy].num_obstacles + 10))))
					{
						// Wall has vanished
						w->pages[pagex][pagey]->units[offsx][offsy].result &= ~(UNIT_WALL);
					}
				}
				w->changed[pagex][pagey] = 1;
			}
		}
	}

	if(avg_drift_cnt > 50)
	{
		*after_dx = (avg_drift_x*MAP_UNIT_W)/avg_drift_cnt;
		*after_dy = (avg_drift_y*MAP_UNIT_W)/avg_drift_cnt;
	}

	printf("INFO: Average adjustment during map insertion: x=%d mm, y=%d mm (%d samples)\n", *after_dx, *after_dy, avg_drift_cnt);


	free(temp_map);
	return 0;
}

void lidars_avg_midpoint(int n_lidars, lidar_scan_t** lidar_list, int32_t* mid_x, int32_t* mid_y)
{
	int64_t x = 0;
	int64_t y = 0;
	for(int l=0; l<n_lidars; l++)
	{
		lidar_scan_t* lid = lidar_list[l];
		x += lid->robot_pos.x;
		y += lid->robot_pos.y;
	}

	*mid_x = x / n_lidars;
	*mid_y = y / n_lidars;
}

/*
map_lidars takes a set of lidar scans, assumes they are in sync (i.e., robot coordinates relative
between the images are correct enough), searches for the map around expected coordinates to find
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

int bigger_search_area = 0;

void map_next_with_larger_search_area()
{
	if(bigger_search_area == 0)
		bigger_search_area = 1;
}

void map_next_with_extra_large_search_area()
{
	bigger_search_area = 2;
}

static int do_map_lidars(world_t* w, int n_lidars, lidar_scan_t** lidar_list, int* da, int* dx, int* dy)
{
	*da = 0;
	*dx = 0;
	*dy = 0;

	if(n_lidars > 32)
	{
		printf("Error: n_lidars must be <=32\n");
		return -1;
	}

//	FILE* fdbg = fopen("map_verbose.csv", "w");
//	if(!fdbg)
//	{
//		printf("Error: couldn't open map_verbose.csv for write.\n");
//		return 1;
//	}

	printf("Info: Attempting to map %d lidar images\n", n_lidars);

	prefilter_lidar_list(n_lidars, lidar_list);


	int mid_x, mid_y;

	// Calculate average robot coordinates between the images, to find arithmetical midpoint.
	// When correcting angle, image is rotated around this point.
	lidars_avg_midpoint(n_lidars, lidar_list, &mid_x, &mid_y);

//	fprintf(fdbg, "PASS 1\nda;dx;dy;score;match_walls;exacts;new_walls;discovered_walls\n");

	int a_range = 2;
	int x_range = 320;
	int y_range = 320;
	int a_step = 1*ANG_1_DEG;

	if(bigger_search_area == 1)
	{
		printf("INFO: Using bigger search area, this will take longer\n");
		a_range = 4;
		x_range = 400;
		y_range = 400;
		a_step = 1*ANG_1_DEG;
	}
	else if(bigger_search_area == 2)
	{
		printf("INFO: Using MUCH bigger search area, this will take quite long\n");
		a_range = 18;
		x_range = 560;
		y_range = 560;
		a_step = 2*ANG_1_DEG;
	}
	else if(bigger_search_area == 3)
	{
		printf("INFO: Using MASSIVE search area, this will take ages\n");
		a_range = 18;
		x_range = 560;
		y_range = 560;
		a_step = 2*ANG_1_DEG;
	}

	int best_score = -999999;
	int best1_da=0, best1_dx=0, best1_dy=0, best_matched=0;
	for(int ida=-1*a_range*ANG_1_DEG; ida<=a_range*ANG_1_DEG; ida+=a_step)
	{
		for(int idx=-1*x_range; idx<=x_range; idx+=80)
		{
			for(int idy=-1*y_range; idy<=y_range; idy+=80)
			{
				int n_matched_walls=0, n_exactly_matched_walls=0, n_new_walls=0, n_discovered_walls=0;
				int score_now = score(w, n_lidars, lidar_list, 
					ida, idx, idy, mid_x, mid_y,
					&n_matched_walls, &n_exactly_matched_walls, &n_new_walls, &n_discovered_walls);

//				fprintf(fdbg, "%.2f;%d;%d;%d;%d;%d;%d;%d\n",
//					(float)ida/(float)ANG_1_DEG, idx, idy, score_now, n_matched_walls, n_exactly_matched_walls, n_new_walls, n_discovered_walls);

				if(score_now > best_score)
				{
					best_score = score_now;
					best_matched = n_matched_walls;
					best1_da = ida;
					best1_dx = idx;
					best1_dy = idy;
				}
			}
		}
	}

	int best_da = 0;
	int best_dx = 0;
	int best_dy = 0;

	int do_not_map = 0;
	if(best_score < 4000)
	{
		if(best_matched == 0) // zero matched walls
		{
			printf("Info: area seems unmapped, and is being mapped with no correction.\n");
			bigger_search_area = 0;
		}
		else
		{
			printf("Info: best score (%d) is so low that we are clearly lost! Mapping is prevented.\n", best_score);
			do_not_map = 1;
			if(bigger_search_area > 1)
				return 6;
			bigger_search_area++;
			return 5;
		}
	}
	else
	{

		if(best1_da == -1*a_range*ANG_1_DEG || best1_da == a_range*ANG_1_DEG || best1_dx == -1*x_range || 
		   best1_dx == 1*x_range || best1_dy == -1*y_range || best1_dy == 1*y_range)
		{
			printf("INFO: Best score was the edge case: must increase search range.\n");
			do_not_map = 1;
			bigger_search_area++;
			return 5;
		}

		bigger_search_area = 0;
//		fprintf(fdbg, "\nPASS 2\nda;dx;dy;score;match_walls;exacts;new_walls;discovered_walls\n");

		best_score = -999999;
		int best2_da=0, best2_dx=0, best2_dy=0;
		for(int ida=best1_da-2*ANG_0_5_DEG; ida<=best1_da+2*ANG_0_5_DEG; ida+=ANG_0_5_DEG)
		{
			for(int idx=best1_dx-80; idx<=best1_dx+80; idx+=40)
			{
				for(int idy=best1_dy-80; idy<=best1_dy+80; idy+=40)
				{
					int n_matched_walls=0, n_exactly_matched_walls=0, n_new_walls=0, n_discovered_walls=0;
					int score_now = score(w, n_lidars, lidar_list, 
						ida, idx, idy, mid_x, mid_y,
						&n_matched_walls, &n_exactly_matched_walls, &n_new_walls, &n_discovered_walls);

//					fprintf(fdbg, "%.2f;%d;%d;%d;%d;%d;%d;%d\n",
//						(float)ida/(float)ANG_1_DEG, idx, idy, score_now, n_matched_walls, n_exactly_matched_walls, n_new_walls, n_discovered_walls);

					if(score_now > best_score)
					{
						best_score = score_now;
						best2_da = ida;
						best2_dx = idx;
						best2_dy = idy;
					}
				}
			}
		}

		best_da = best2_da;
		best_dx = best2_dx;
		best_dy = best2_dy;
	}

	if(!do_not_map)
	{
		printf("Info: Map search complete, correction a=%.1fdeg, x=%dmm, y=%dmm, score=%d\n", (float)best_da/(float)ANG_1_DEG, best_dx, best_dy, best_score);

		int32_t aft_corr_x = 0, aft_corr_y = 0;

		do_mapping(w, n_lidars, lidar_list, best_da, best_dx, best_dy, mid_x, mid_y, &aft_corr_x, &aft_corr_y);

		*da = best_da;
		*dx = best_dx + aft_corr_x;
		*dy = best_dy + aft_corr_y;
	}

//	fclose(fdbg);

	return 0;
}


int search_area_size = 0;

void massive_search_area()
{
	search_area_size = 1;
}

int do_map_lidars_new_quick(world_t* w, int n_lidars, lidar_scan_t** lidar_list, int* da, int* dx, int* dy)
{
	static int8_t scoremap[TEMP_MAP_W*TEMP_MAP_W];

	*da = 0;
	*dx = 0;
	*dy = 0;

	printf("Info: Attempting to map %d lidar images\n", n_lidars);

	if(n_lidars > 32)
	{
		printf("Error: n_lidars must be <=32\n");
		return -1;
	}

	prefilter_lidar_list(n_lidars, lidar_list);

	int mid_x, mid_y;

	// Calculate average robot coordinates between the images, to find arithmetical midpoint.
	// When correcting angle, image is rotated around this point.
	lidars_avg_midpoint(n_lidars, lidar_list, &mid_x, &mid_y);

	if(search_area_size)
		gen_scoremap_for_large_steps(w, scoremap, mid_x, mid_y);
	else
		gen_scoremap_for_small_steps(w, scoremap, mid_x, mid_y);

	int a_range, xy_range, xy_step, a_step;

	if(search_area_size == 0)
	{
		a_range = 4;
		xy_range = 360;
		xy_step = 40;
		a_step = 1*ANG_1_DEG;
	}
	else
	{
		a_range = 24;
		xy_range = 1800;
		xy_step = 120;
		a_step = 2;
	}

	int n_xy_steps = 2*(xy_range/xy_step) + 1;

	int best_score = -999999;
	int best1_da=0, best1_dx=0, best1_dy=0;

	for(int ida=-1*a_range*ANG_1_DEG; ida<=a_range*ANG_1_DEG; ida+=a_step)
	{
		int32_t idx = 0, idy = 0;
		int score_now = score_quick_search_xy(scoremap, n_lidars, lidar_list, mid_x, mid_y,
			ida, -1*xy_range, xy_step, n_xy_steps, -1*xy_range, xy_step, n_xy_steps, &idx, &idy);
		if(score_now > best_score)
		{
			best_score = score_now;
			best1_da = ida;
			best1_dx = idx;
			best1_dy = idy;
		}
	}

	int pass2_a_range, pass2_a_step;
	int pass2_dx_start, pass2_dx_step, pass2_num_dx, pass2_dy_start, pass2_dy_step, pass2_num_dy;

	if(search_area_size == 0)
	{
		// we already generated scoremap for small steps
		pass2_a_range = 2; // in half degs
		pass2_a_step = ANG_0_5_DEG;
		pass2_dx_start = best1_dx-40;
		pass2_dx_step = 20;
		pass2_num_dx = 2*(40/20) + 1;
		pass2_dy_start = best1_dy-40;
		pass2_dy_step = 20;
		pass2_num_dy = 2*(40/20) + 1;
	}
	else
	{
		printf("Info: Pass1 complete, correction a=%.1fdeg, x=%dmm, y=%dmm, score=%d\n", (float)best1_da/(float)ANG_1_DEG, best1_dx, best1_dy, best_score);

		gen_scoremap_for_small_steps(w, scoremap, mid_x, mid_y); // overwrite large step scoremap.
		pass2_a_range = 4; // in half degs
		pass2_a_step = ANG_0_5_DEG;
		pass2_dx_start = best1_dx-120;
		pass2_dx_step = 20;
		pass2_num_dx = 2*(120/20) + 1;
		pass2_dy_start = best1_dy-120;
		pass2_dy_step = 20;
		pass2_num_dy = 2*(120/20) + 1;
	}

	best_score = -999999;
	int best2_da=0, best2_dx=0, best2_dy=0;

	for(int ida=best1_da-pass2_a_range*pass2_a_step; ida<=best1_da+pass2_a_range*pass2_a_step; ida+=pass2_a_step)
	{
		int32_t idx = 0, idy = 0;
		int score_now = score_quick_search_xy(scoremap, n_lidars, lidar_list, mid_x, mid_y,
			ida, pass2_dx_start, pass2_dx_step, pass2_num_dx, pass2_dy_start, pass2_dy_step, pass2_num_dy, &idx, &idy);
		if(score_now > best_score)
		{
			best_score = score_now;
			best2_da = ida;
			best2_dx = idx;
			best2_dy = idy;
		}
	}

	int best_da = best2_da;
	int best_dx = best2_dx;
	int best_dy = best2_dy;

	printf("Info: Map search complete, correction a=%.1fdeg, x=%dmm, y=%dmm, score=%d\n", (float)best_da/(float)ANG_1_DEG, best_dx, best_dy, best_score);

	if(best_score < 1000)
	{
		printf("INFO: Best score low, mapping with zero correction.\n");
		best_da = 0; best_dx = 0; best_dy = 0;
	}

	int32_t aft_corr_x = 0, aft_corr_y = 0;

	do_mapping(w, n_lidars, lidar_list, best_da, best_dx, best_dy, mid_x, mid_y, &aft_corr_x, &aft_corr_y);

	*da = best_da;
	*dx = best_dx + aft_corr_x;
	*dy = best_dy + aft_corr_y;

	return 0;


}

int map_lidars(world_t* w, int n_lidars, lidar_scan_t** lidar_list, int* da, int* dx, int* dy)
{
	int ret = -1;
//	if( ( ret = do_map_lidars(w, n_lidars, lidar_list, da, dx, dy) ) == 5)
//	{
//		if( ( ret = do_map_lidars(w, n_lidars, lidar_list, da, dx, dy) ) == 5)
//			ret = do_map_lidars(w, n_lidars, lidar_list, da, dx, dy);
//	}

	ret = do_map_lidars_new_quick(w, n_lidars, lidar_list, da, dx, dy);
	return ret;
}

int map_lidar_to_minimap(lidar_scan_t *p_lid)
{
	if(!p_lid)
	{
		printf("ERROR: invalid p_lid\n");
		return -1;
	}

//	printf("Info: mapping lidar to minimap\n");
	memset(minimap, 0, MINIMAP_SIZE*(MINIMAP_SIZE/32+1)*sizeof(uint32_t));
	for(int p=0; p<LIDAR_SCAN_POINTS; p++)
	{
		if(!p_lid->scan[p].valid)
			continue;

		int x = (p_lid->scan[p].x - p_lid->robot_pos.x) / MAP_UNIT_W + MINIMAP_MIDDLE;
		int y = (p_lid->scan[p].y - p_lid->robot_pos.y) / MAP_UNIT_W + MINIMAP_MIDDLE;

		int yoffs = y/32;
		int yoffs_remain = y - yoffs*32;		

		if(x < 0 || x >= MINIMAP_SIZE || yoffs < 0 || yoffs >= MINIMAP_SIZE/32+1)
		{
			printf("WARN: ignoring out of range coordinates (map_lidar_to_minimap(), x=%d, yoffs=%d, yoffs_remain=%d)\n", x, yoffs, yoffs_remain);
			continue;
		}

		minimap[x][yoffs] |= 1<<(32-yoffs_remain);
	}

	return 0;
}

int map_lidars_to_minimap(int n_lidars, lidar_scan_t** lidar_list)
{
	prefilter_lidar_list_aggressive(n_lidars, lidar_list);
	return map_lidar_to_minimap(lidar_list[n_lidars-1]);
}

const int robot_xs = 480;
const int robot_ys = 524;
const int lidar_xoffs = 120;
const int lidar_yoffs = 0;

#define STOP_REASON_JERK 1
#define STOP_REASON_OBSTACLE_FRONT_RIGHT 2
#define STOP_REASON_OBSTACLE_FRONT_LEFT 3
#define STOP_REASON_OBSTACLE_BACK_LEFT 5
#define STOP_REASON_OBSTACLE_BACK_RIGHT 6


#define ORIGIN_TO_ROBOT_FRONT 130
#define ASSUMED_ITEM_POS_FROM_MIDDLE_START 100
#define ASSUMED_ITEM_STEP_SIZE (MAP_UNIT_W)
#define ASSUMED_ITEM_NUM_STEPS 5

const int robot_outline[32] =
{
100,
(100+170)/2,
170,
(170+200)/2,
200,
(200+220)/2,
220,
(220+200)/2,
200,
(200+250)/2,
250,
(250+300)/2,
300,
(300+390)/2,
390,
(390+350)/2,
350,
(350+390)/2,
390,
(390+300)/2,
300,
(300+250)/2,
250,
(250+200)/2,
200,
(200+220)/2,
220,
(220+200)/2,
200,
(200+170)/2,
170,
(170+100)/2
};


void map_collision_obstacle(world_t* w, int32_t cur_ang, int cur_x, int cur_y, int stop_reason, int vect_valid, float vect_ang_rad)
{
	int idx_x, idx_y, offs_x, offs_y;
	if(stop_reason == STOP_REASON_OBSTACLE_FRONT_LEFT || stop_reason == STOP_REASON_OBSTACLE_FRONT_RIGHT)
	{
		printf("Mapping FRONT obstacle due to wheel slip.\n");
		float x = (float)cur_x + cos(ANG32TORAD(cur_ang))*(float)(ORIGIN_TO_ROBOT_FRONT+20.0);
		float y = (float)cur_y + sin(ANG32TORAD(cur_ang))*(float)(ORIGIN_TO_ROBOT_FRONT+20.0);

		// Shift the result to the right or left:
		int32_t angle = cur_ang + (uint32_t)( ((stop_reason==STOP_REASON_OBSTACLE_FRONT_RIGHT)?90:-90) *ANG_1_DEG);

		x += cos(ANG32TORAD(angle))*(float)ASSUMED_ITEM_POS_FROM_MIDDLE_START;
		y += sin(ANG32TORAD(angle))*(float)ASSUMED_ITEM_POS_FROM_MIDDLE_START;

		for(int i = 0; i < ASSUMED_ITEM_NUM_STEPS; i++)
		{
			x += cos(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;
			y += sin(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;

			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
			PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
			PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
			PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
			w->changed[idx_x][idx_y] = 1;
		}
	}

	else if(stop_reason == STOP_REASON_OBSTACLE_BACK_LEFT || stop_reason == STOP_REASON_OBSTACLE_BACK_RIGHT)
	{
		printf("Mapping BACK (ARSE) obstacle due to wheel slip.\n");
		float x = (float)cur_x + cos(ANG32TORAD(cur_ang))*(float)-300.0;
		float y = (float)cur_y + sin(ANG32TORAD(cur_ang))*(float)-300.0;

		// Shift the result to the right or left:
		int32_t angle = cur_ang + (uint32_t)( ((stop_reason==STOP_REASON_OBSTACLE_BACK_RIGHT)?90:-90) *ANG_1_DEG);

		x += cos(ANG32TORAD(angle))*(float)robot_ys/2.0;
		y += sin(ANG32TORAD(angle))*(float)robot_ys/2.0;

		for(int i = 0; i < 2; i++)
		{
			x += cos(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;
			y += sin(ANG32TORAD(angle))*(float)ASSUMED_ITEM_STEP_SIZE;

			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
			PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
			PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
			PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
			w->changed[idx_x][idx_y] = 1;
		}
	}

	else if(stop_reason == STOP_REASON_JERK)
	{
/*		printf("Mapping obstacle due to acceleration (ang = %.0f).\n", RADTODEG(vect_ang_rad));
		for(int i=-2; i<=2; i++)
		{
			int idx = 32.0*vect_ang_rad/(2.0*M_PI);
			idx += i;
			while(idx < 0) idx+=32;
			while(idx > 31) idx-=32;
			for(int o = 0; o < 2; o++)
			{
				int dist_to_outline = robot_outline[idx] + 30 + o*40;
				float x = (float)cur_x + cos( ANG32TORAD(cur_ang) + vect_ang_rad+((float)i*2.0*M_PI/32.0) )*(float)dist_to_outline;
				float y = (float)cur_y + sin( ANG32TORAD(cur_ang) + vect_ang_rad+((float)i*2.0*M_PI/32.0) )*(float)dist_to_outline;

				page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
				load_9pages(&world, idx_x, idx_y);
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest |= UNIT_ITEM | UNIT_WALL | UNIT_DO_NOT_REMOVE_BY_LIDAR;
				PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
				PLUS_SAT_255(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
				w->changed[idx_x][idx_y] = 1;
			}
		} */
	}
	else
	{
		printf("WARN: Unrecognized stop reason %d\n", stop_reason);
	}
	

}

void clear_within_robot(world_t* w, pos_t pos)
{
	int idx_x, idx_y, offs_x, offs_y;
	for(int stripe = 0; stripe < robot_xs/20 - 1; stripe++)
	{
		float x = (float)pos.x + cos(ANG32TORAD(pos.ang))*(float)(ORIGIN_TO_ROBOT_FRONT-stripe*20);
		float y = (float)pos.y + sin(ANG32TORAD(pos.ang))*(float)(ORIGIN_TO_ROBOT_FRONT-stripe*20);

		// Shift the result in robot_y direction
		int32_t angle = pos.ang + (uint32_t)(90*ANG_1_DEG);

		x += cos(ANG32TORAD(angle))*((float)robot_ys/-2.0+10);
		y += sin(ANG32TORAD(angle))*((float)robot_ys/-2.0+10);

		for(int i = 0; i < robot_ys/20 - 1; i++)
		{
			x += cos(ANG32TORAD(angle))*(float)20.0;
			y += sin(ANG32TORAD(angle))*(float)20.0;

			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_1page(&world, idx_x, idx_y);
			if((world.pages[idx_x][idx_y]->units[offs_x][offs_y].result & UNIT_WALL) ||
			   (world.pages[idx_x][idx_y]->units[offs_x][offs_y].result & UNIT_ITEM))
			{
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].result = UNIT_MAPPED;
				world.pages[idx_x][idx_y]->units[offs_x][offs_y].latest = UNIT_MAPPED;
				MINUS_SAT_0(world.pages[idx_x][idx_y]->units[offs_x][offs_y].num_obstacles);
				w->changed[idx_x][idx_y] = 1;
			}
		}
	}
}


void map_sonar(world_t* w, sonar_scan_t* p_son)
{
	int idx_x, idx_y, offs_x, offs_y;

	// Erase old items, but only if all three sonars show a ping from farther away.

	if(p_son->scan[0].valid && p_son->scan[1].valid && p_son->scan[2].valid)
	{
		float nearest = 2000.0;
		for(int i = 0; i < 3; i++)
		{
			int dx = p_son->scan[i].x - p_son->robot_pos.x;
			int dy = p_son->scan[i].y - p_son->robot_pos.y;

			float cur_len = sqrt(sq(dx) + sq(dy));

			if(cur_len < nearest) nearest = cur_len;
		}

		if(nearest > 500.0)
		{
			const float step = 3*MAP_UNIT_W;

			float pos = 300.0;
			int terminate = 0;

			int dx = p_son->scan[1].x - p_son->robot_pos.x;
			int dy = p_son->scan[1].y - p_son->robot_pos.y;
			float ang = atan2(dy, dx) + M_PI;
			if(ang < 0.0) ang += 2.0*M_PI;
			else if(ang > 2.0*M_PI) ang -= 2.0*M_PI;

		//	printf("INFO: Clearing items start (%d, %d) ang = %.1f deg, len = %.1f\n", p_son->scan[1].x, p_son->scan[1].y, RADTODEG(ang), nearest);
		//	printf("ang = %.4f  dir = %d \n", ang, dir);

			while(1)
			{
				int x = (cos(ang)*pos + (float)p_son->scan[1].x);
				int y = (sin(ang)*pos + (float)p_son->scan[1].y);

				for(int ix=-5*MAP_UNIT_W; ix<=5*MAP_UNIT_W; ix+=MAP_UNIT_W)
				{
					for(int iy=-5*MAP_UNIT_W; iy<=5*MAP_UNIT_W; iy+=MAP_UNIT_W)
					{	
						page_coords(x+ix,y+iy, &idx_x, &idx_y, &offs_x, &offs_y);
						load_9pages(&world, idx_x, idx_y);
						world.pages[idx_x][idx_y]->units[offs_x][offs_y].result &= ~(UNIT_ITEM);
					}
				}

				if(terminate) break;
				pos += step;
				if(pos > nearest-150)
				{
					pos = nearest;
					terminate = 1;
				}
			}
		}
	}

	for(int i=0; i<3; i++)
	{
		if(!p_son->scan[i].valid) continue;

		for(int s=0; s<25; s++)
		{
			int x = p_son->scan[i].x+search_order[s][0]*MAP_UNIT_W;
			int y = p_son->scan[i].y+search_order[s][1]*MAP_UNIT_W;
			page_coords(x,y, &idx_x, &idx_y, &offs_x, &offs_y);
			load_9pages(&world, idx_x, idx_y);

			if(world.pages[idx_x][idx_y]->units[offs_x][offs_y].result & UNIT_ITEM)
			{
//				printf("INFO: Item already mapped\n");
				goto ALREADY_MAPPED_ITEM;
			}
		}

		int dx = p_son->scan[i].x - p_son->robot_pos.x;
		int dy = p_son->scan[i].y - p_son->robot_pos.y;

		int sqdist = sq(dx) + sq(dy);

		if(sqdist < sq(1500))
		{
			page_coords(p_son->scan[i].x,p_son->scan[i].y, &idx_x, &idx_y, &offs_x, &offs_y);
			world.pages[idx_x][idx_y]->units[offs_x][offs_y].result |= UNIT_ITEM;
//			printf("INFO: Mapping an item\n");
			//world.changed[idx_x][idx_y] = 1;
		}

		ALREADY_MAPPED_ITEM: ;
	}
}

int unfamiliarity_score(world_t* w, int x, int y)
{
	int n_walls = 0;
	int n_seen = 0;
	int n_visited = 1; // to avoid div per zero
	for(int xx = x - 400; xx <= x + 400; xx += 40)
	{
		for(int yy = y - 400; yy <= y + 400; yy += 40)
		{
			int px, py, ox, oy;
			page_coords(xx,yy, &px, &py, &ox, &oy);

			if(w->pages[px][py])
			{
				if(w->pages[px][py]->units[ox][oy].result & UNIT_WALL) n_walls++;
				if(n_walls > 1)
					return 0;
				n_seen += w->pages[px][py]->units[ox][oy].num_seen;
				n_visited += w->pages[px][py]->units[ox][oy].num_visited;
			}

		}
	}

	return (100*sqrt(sqrt(n_seen)))/n_visited;
}

typedef struct
{
	int enabled;
	int x;
	int y;
} cant_goto_place_t;

// This list is populated when going towards a (desired_x,desired_y) destination leads to stop, or the dest is not reached within sensible time.
// Places near the ones in this list are ignored when finding unfamiliar places.
// The list is purposedly overwritten so that we try again older places, in case the conditions have changed.
// TODO: move this to work per every world.

#define CANT_GOTO_PLACE_LIST_LEN 16
cant_goto_place_t cant_goto_places[CANT_GOTO_PLACE_LIST_LEN];
int cant_goto_place_wr_idx;

void add_cant_goto_place(int x, int y)
{
	printf("INFO: Adding cant_goto_place idx=%d, abs (%d, %d) mm\n", cant_goto_place_wr_idx, x, y);
	cant_goto_places[cant_goto_place_wr_idx].enabled = 1;
	cant_goto_places[cant_goto_place_wr_idx].x = x;
	cant_goto_places[cant_goto_place_wr_idx].y = y;
	cant_goto_place_wr_idx++;
	if(cant_goto_place_wr_idx >= CANT_GOTO_PLACE_LIST_LEN)
	{
		printf("INFO: cant_goto_place_list wraps around, old out-of-limits areas will be tried again.\n");
		cant_goto_place_wr_idx = 0;

	}
}

int find_unfamiliar_direction(world_t* w, int *x_out, int *y_out)
{
	int biggest = 0;
	int biggest_x, biggest_y;
	for(int dx = -4800; dx <= 4800; dx += 200)
	{
		for(int dy = -4800; dy <= 4800; dy += 200)
		{
			extern int32_t cur_x, cur_y;
			int score = unfamiliarity_score(w, cur_x+dx, cur_y+dy);
			if(score > biggest)
			{
				for(int i = 0; i < CANT_GOTO_PLACE_LIST_LEN; i++)
				{
					if(cant_goto_places[i].enabled && (sq(cant_goto_places[i].x-(cur_x+dx))+sq(cant_goto_places[i].x-(cur_x+dx))) < sq(300) )
					{
						printf("Info: ignoring potential biggest score place, cant_goto_idx=%d, abs (%d, %d) mm.\n", i, cur_x+dx, cur_y+dy);
						goto CONTINUE_UNFAM_LOOP;
					}
				}

				biggest = score;
				biggest_x = cur_x+dx;
				biggest_y = cur_y+dy;

				CONTINUE_UNFAM_LOOP:;
			}
		}
	}

	if(biggest)
	{
		*x_out = biggest_x;
		*y_out = biggest_y;
	}

	return biggest;
}

const char* const AUTOSTATE_NAMES[] =
{
	"IDLE",
	"START",
	"COMPASS",
	"WAIT_COMPASS_START",
	"WAIT_COMPASS_END",
	"SYNC_TO_COMPASS",
	"GEN_DESIRED_DIR",
	"FIND_DIR",
	"WAIT_MOVEMENT",
	"DAIJUING",
	"res",
	"res",
	"res",
	"res",
	"res"
};

typedef enum
{
	S_IDLE   		= 0,
	S_START 		= 1,
	S_COMPASS		= 2,
	S_WAIT_COMPASS_START	= 3,
	S_WAIT_COMPASS_END	= 4,
	S_SYNC_TO_COMPASS	= 5,
	S_GEN_DESIRED_DIR	= 6,
	S_FIND_DIR		= 7,
	S_WAIT_MOVEMENT  	= 8,
	S_DAIJUING		= 9
} autostate_t;

autostate_t cur_autostate;

void start_automapping_from_compass()
{
	cur_autostate = S_START;
}

void start_automapping_skip_compass()
{
	cur_autostate = S_GEN_DESIRED_DIR;
}

void stop_automapping()
{
	map_significance_mode = MAP_SIGNIFICANT_IMGS;
	cur_autostate = S_IDLE;
}

int automap_only_compass;

void start_automap_only_compass()
{
	cur_autostate = S_START;
	automap_only_compass = 1;
}

void autofsm()
{
	static int movement_id = 0;
	extern int32_t cur_compass_ang;
	extern int compass_round_active;
	extern int32_t cur_x, cur_y;
	extern int mapping_on;

	int prev_autostate = cur_autostate;

	static int desired_x;
	static int desired_y;
	static int same_dir_cnt;
	static int same_dir_len;
	static int daijuing_cnt;
	static int num_stops;

	switch(cur_autostate)
	{
		case S_IDLE: {

		} break;

		case S_START: {
			daiju_mode(0);
			mapping_on = 0;
			cur_autostate++;
		} break;

		case S_COMPASS: {
			printf("INFO: Started compass round\n");
			do_compass_round();
			cur_autostate++;
		} break;

		case S_WAIT_COMPASS_START: {
			if(compass_round_active)
				cur_autostate++;
		} break;

		case S_WAIT_COMPASS_END: {
			if(!compass_round_active)
				cur_autostate++;
		} break;

		case S_SYNC_TO_COMPASS: {
			int32_t ang = cur_compass_ang-90*ANG_1_DEG;
			set_robot_pos(ang,cur_x,cur_y);		
			printf("INFO: Syncing robot angle to compass, turning mapping on, requesting massive search area.\n");
			mapping_on = 1;
			massive_search_area();
			if(automap_only_compass)
				cur_autostate = S_IDLE;
			else
				cur_autostate++;
		} break;

		case S_GEN_DESIRED_DIR: {
			daiju_mode(0);

			int unfam_score = find_unfamiliar_direction(&world, &desired_x, &desired_y);

			if(unfam_score)
			{
				same_dir_len = 10;
				printf("INFO: Generated new desired vector abs (%d, %d) mm based on unfamiliarity score %d, time to follow = %d\n", desired_x, desired_y, unfam_score, same_dir_len);
				if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 0, 255, 40, 0);
			}
			else
			{
				float rand1 = ((float)rand() / (float)RAND_MAX)*4000.0+2000.0;
				float rand2 = ((float)rand() / (float)RAND_MAX)*4000.0+2000.0;

				if(rand()&1)
					rand1 *= -1;
				if(rand()&1)
					rand2 *= -1;

				desired_x = cur_x+rand1;
				desired_y = cur_y+rand2;
				same_dir_len = 10; //((float)rand() / (float)RAND_MAX)*7.0;
				printf("INFO: No unfamiliarity scores generated (area unmapped?): generated new random desired vector abs (%d, %d) mm, time to follow = %d\n", desired_x, desired_y, same_dir_len);
				if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 128, 150, 40, 0);
			}
			same_dir_cnt = 0;
			cur_autostate++;
		} break;

		case S_FIND_DIR: {
			map_significance_mode = MAP_SIGNIFICANT_IMGS | MAP_SEMISIGNIFICANT_IMGS;
			mapping_on = 1;

			#define NUM_LATEST_LIDARS_FOR_ROUTING_START 7
			extern lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];
			map_lidars_to_minimap(NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start);

//			map_lidar_to_minimap(latest_lidar);

			int32_t dx, dy;
			int need_to_back = 0;
			extern int32_t cur_ang;
			if(num_stops > 8)
			{
				num_stops = 0;
				printf("INFO: Too many stops without success, daijuing for a while.\n");
				daiju_mode(1);
				cur_autostate = S_DAIJUING;
				daijuing_cnt = 0;
			}
			else
			{
				int ret;
				if( (ret = minimap_find_mapping_dir(&world, ANG32TORAD(cur_ang), &dx, &dy, desired_x-cur_x, desired_y-cur_y, &need_to_back)) )
				{
					printf("INFO: Found direction\n");
					if(movement_id == cur_xymove.id)
					{
						printf("INFO: id issue, incrementing by 5\n");
						movement_id+=5;
						if(movement_id > 100) movement_id = 0;
					}
					set_hw_obstacle_avoidance_margin((ret&2)?0:120);

					move_to(cur_x+dx, cur_y+dy, need_to_back, movement_id, 30, 0);
					cur_autostate++;
				}
				else
				{
					printf("INFO: Automapping: can't go anywhere; daijuing for a while.\n");
					daiju_mode(1);
					cur_autostate = S_DAIJUING;
					daijuing_cnt = 0;
				}
			}

		} break;

		case S_WAIT_MOVEMENT: {

			if(cur_xymove.id == movement_id && cur_xymove.remaining < 80)
			{
				num_stops = 0;
				printf("INFO: Automapping: movement id=%d finished, ", movement_id);

				movement_id++; if(movement_id > 100) movement_id = 0;
				same_dir_cnt++;
				if(same_dir_cnt > same_dir_len || (sq(cur_x-desired_x) + sq(cur_y-desired_y)) < sq(300))
				{
					printf("generate new desired direction.\n");
					if(same_dir_cnt > same_dir_len)
					{
						add_cant_goto_place(desired_x, desired_y);
						if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 255, 30, 50, 1);
					}
					cur_autostate = S_GEN_DESIRED_DIR;
				}
				else
				{
					printf("continue with the old desired direction.\n");
					cur_autostate = S_FIND_DIR;
				}
			}
			else if(cur_xymove.id == movement_id && (cur_xymove.micronavi_stop_flags || cur_xymove.feedback_stop_flags))
			{
				printf("INFO: Automapping: movement id=%d stopped, generating new desired direction\n", movement_id);
				add_cant_goto_place(desired_x, desired_y);
				if(tcp_client_sock >= 0) tcp_send_dbgpoint(desired_x, desired_y, 255, 30, 50, 1);
				movement_id++; if(movement_id > 100) movement_id = 0;
				num_stops++;
				cur_autostate = S_GEN_DESIRED_DIR;
			}

		} break;

		case S_DAIJUING: {
			if(++daijuing_cnt > 50000)
			{
				cur_autostate = S_GEN_DESIRED_DIR;
				daiju_mode(0);
			}
		} break;

		default: break;
	}

	if(cur_autostate != prev_autostate)
	{
		printf("INFO: autostate change %s --> %s\n", AUTOSTATE_NAMES[prev_autostate], AUTOSTATE_NAMES[cur_autostate]);
	}
}


void dbg_test()
{
	extern lidar_scan_t* lidars_to_map_at_routing_start[NUM_LATEST_LIDARS_FOR_ROUTING_START];
	map_lidars_to_minimap(NUM_LATEST_LIDARS_FOR_ROUTING_START, lidars_to_map_at_routing_start);
	int32_t dx, dy;
	int need_to_back = 0;
	extern int32_t cur_ang;
	if(minimap_find_mapping_dir(&world, ANG32TORAD(cur_ang), &dx, &dy, 0, 0, &need_to_back))
	{
		printf("DBG_TEST: Found direction dx=%d  dy=%d  need_to_back=%d\n", dx, dy, need_to_back);
	}
	else
	{
		printf("DBG_TEST: Automapping: can't go anywhere; daijuing needed.\n");
	}
}
