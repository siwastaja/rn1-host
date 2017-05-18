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

int tcp_parser(int sock)
{
	uint8_t dummy[1024];
	int ret;
	static int state = 0; // Num of bytes read
	static struct header __attribute__ ((packed)) { uint8_t mid; uint16_t size; }; // Stored header of incoming message
	static int bytes_left = 0;
	static tcp_message_t* msg = 0; // Once message is recognized, pointer to message type struct.
	static int unrecog = 0;

	static int cur_field = 0;

	if(state < 3)
	{
		msg = 0;
		unrecog = 0;
		cur_field = 0;
		ret = read(sock, (uint8_t*)header + (uint8_t*)state, 3-state);
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: TCP stream read error %d (%s)\n", errno, strerror(errno));
		}
		else if(ret == 0)
		{
			fprintf(stderr, "ERROR: TCP stream read() returned 0 bytes even when it shoudln't.\n");
		}
		else
			state += ret;
	}
	else
	{
		if(msg == 0 && unrecog == 0)
		{		
			for(int i=0; i<NUM_CR_MSGS; i++)
			{
				if(CR_MSGS[i].mid == header.mid)
				{
					msg = CR_MSGS[i];
					break;
				}
			}

			if(!msg)
			{
				fprintf(stderr, "WARN: Ignoring unrecognized message with msgid 0x%02x\n", mid);
				unrecog = 1;
			}
			else if(header.size != CR_MSGS[i].size)
			{
				fprintf(stderr, "WARN: Ignoring message with msgid 0x%02x because of size mismatch (got:%u, expected:%u)\n",
					mid, header.size, CR_MSGS[i].size);
				unrecog = 1;
			}
			bytes_left = header.size;
		}

		if(unrecog) // Read as much data as the size field shows.
		{
			if(bytes_left)
			{
				ret = read(sock, dummy, bytes_left>1024?1024:bytes_left);
				if(ret < 0)
				{
					fprintf(stderr, "ERROR: TCP stream read error %d (%s)\n", errno, strerror(errno));
				}
				else if(ret == 0)
				{
					fprintf(stderr, "ERROR: TCP stream read() returned 0 bytes even when it shoudln't.\n");
				}
				else
					bytes_left -= ret;
			}

			if(!bytes_left)
			{
				state = 0;
			}

		}
		else // Recognized message, parse it.
		{
			if(msg->types[cur_field]

		}
			

	}

}

