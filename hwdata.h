#ifndef HWDATA_H
#define HWDATA_H

#include <stdint.h>
#include "datatypes.h"

int parse_uart_msg(uint8_t* buf, int len);
lidar_scan_t* get_lidar();


#endif
