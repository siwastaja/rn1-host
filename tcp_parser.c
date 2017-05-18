#include <stdint.h>

#include "tcp_comm.h"

// Client->Robot messages
tcp_message_t msg_cr_dest =
{
	0,
	1,
	8, "ii"
};

#define NUM_CR_MSGS 1

const tcp_message_t* CR_MSGS[NUM_CR_MSGS] =
{
	&msg_cr_dest
};

// Robot->Client messages
tcp_message_t msg_rc_pos =
{
	0,
	129,
	10, "sii"
};


typedef enum
{

} parser_state_t;

void parse_tcp_buffer()
{

	if(tcp_rx_ring_rd == tcp_rx_ring_wr)


	uint8_t mid = tcp_buf[tcp_rx_ring_rd]
	for(int i=0; i<NUM_CR_MSGS; i++)
	{
		if(CR_MSGS[i].mid == 
	}

}

