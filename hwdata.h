#ifndef HWDATA_H
#define HWDATA_H

#include <stdint.h>
#include "datatypes.h"

int parse_uart_msg(uint8_t* buf, int len);
lidar_scan_t* get_basic_lidar();
lidar_scan_t* get_significant_lidar();
extern lidar_scan_t* latest_lidar;


sonar_scan_t* get_sonar();

void send_keepalive();
void release_motors();

void move_to(int32_t x, int32_t y, int8_t backmode, int id, int speedlimit, int accurate_turn);

void correct_robot_pos(int32_t da, int32_t dx, int32_t dy, int id);
void set_hw_obstacle_avoidance_margin(int mm);
void set_robot_pos(int32_t na, int32_t nx, int32_t ny);
void do_compass_round();
void hw_find_charger();
void daiju_mode(int on);
void turn_and_go(int32_t ang_abs, int fwd_rel, int speedlimit, int accurate_turn);
void stop_movement();
void limit_speed(int speedlimit);
void prevent_3dtoffing();


#endif
