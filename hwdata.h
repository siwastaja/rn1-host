#ifndef HWDATA_H
#define HWDATA_H

#include <stdint.h>
#include "datatypes.h"

int parse_uart_msg(uint8_t* buf, int len);
lidar_scan_t* get_basic_lidar();
lidar_scan_t* get_significant_lidar();
sonar_scan_t* get_sonar();


void send_keepalive();
void move_to(int32_t x, int32_t y);



#endif
