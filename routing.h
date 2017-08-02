#ifndef ROUTING_H
#define ROUTING_H

#include "mapping.h"

typedef struct
{
	int x;
	int y;
} route_xy_t;

typedef struct route_unit_t route_unit_t;

struct route_unit_t
{
	route_xy_t loc;
	int backmode;
	route_unit_t* prev;
	route_unit_t* next;
};

void clear_route(route_unit_t **route);
int search_route(world_t *w, route_unit_t **route, float start_ang, int start_x_mm, int start_y_mm, int end_x_mm, int end_y_mm);

#define MINIMAP_SIZE 768
#define MINIMAP_MIDDLE 384
extern uint32_t minimap[MINIMAP_SIZE][MINIMAP_SIZE/32 + 1];


int minimap_find_mapping_dir(world_t *w, float ang_now, int32_t* x, int32_t* y, int32_t desired_x, int32_t desired_y, int* back);

int check_direct_route(int32_t start_ang, int start_x, int start_y, int end_x, int end_y);
int check_direct_route_non_turning(int start_x, int start_y, int end_x, int end_y);
void routing_set_world(world_t *w);
void gen_all_routing_pages(world_t *w, int forgiveness);
void gen_routing_page(world_t *w, int xpage, int ypage, int forgiveness);


#endif
