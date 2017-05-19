
#include <stdint.h>
#include "datatypes.h"
#include "uart.h"

#include "../rn1-brain/comm.h" // For the convenient 7-bit data handling macros.


#define LIDAR_RING_BUF_LEN 8

int lidar_wr = 0;
int lidar_rd = 0;
lidar_scan_t lidars[LIDAR_RING_BUF_LEN];

lidar_scan_t* get_lidar()
{
	if(lidar_wr == lidar_rd)
	{
		return 0;
	}
	
	lidar_scan_t* ret = &lidars[lidar_rd];
	lidar_rd++; if(lidar_rd >= LIDAR_RING_BUF_LEN) lidar_rd = 0;
	return ret;
}

int parse_uart_msg(uint8_t* buf, int len)
{
	switch(buf[0])
	{
		case 0x84:
		{
			lidars[lidar_wr].status = buf[1];
			for(int i = 0; i < 360; i++)
			{
				lidars[lidar_wr].scan[i] = buf[14+2*i+1]<<7 | buf[14+2*i];
			}
			lidars[lidar_wr].pos.ang = (I7I7_U16_lossy(buf[2], buf[3]))<<16;
			lidars[lidar_wr].pos.x = I7x5_I32(buf[4],buf[5],buf[6],buf[7],buf[8]);
			lidars[lidar_wr].pos.y = I7x5_I32(buf[9],buf[10],buf[11],buf[12],buf[13]);
			lidar_wr++; if(lidar_wr >= LIDAR_RING_BUF_LEN) lidar_wr = 0;
		}
		break;
	}

	return 0;
}

void send_keepalive()
{
	uint8_t buf[3] = {0x8f, 0x00, 0xff};
	send_uart(buf, 3);
}

void move_to(int32_t x, int32_t y)
{
	uint8_t buf[12];

	buf[0] = 0x82;
	buf[1] = I32_I7_4(x);
	buf[2] = I32_I7_3(x);
	buf[3] = I32_I7_2(x);
	buf[4] = I32_I7_1(x);
	buf[5] = I32_I7_0(x);
	buf[6] = I32_I7_4(y);
	buf[7] = I32_I7_3(y);
	buf[8] = I32_I7_2(y);
	buf[9] = I32_I7_1(y);
	buf[10] = I32_I7_0(y);
	buf[11] = 0xff;
	send_uart(buf, 12);
}

