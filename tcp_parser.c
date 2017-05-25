#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "datatypes.h"
#include "tcp_comm.h"
#include "tcp_parser.h"

// Client->Robot messages

tcp_cr_dest_t msg_cr_dest;
tcp_message_t msgmeta_cr_dest =
{
	&msg_cr_dest,
	TCP_CR_DEST_MID,
	8, "ii"
};


#define NUM_CR_MSGS 1
tcp_message_t* CR_MSGS[NUM_CR_MSGS] =
{
	&msgmeta_cr_dest
};



// Robot->Client messages
tcp_message_t msgmeta_rc_pos =
{
	0,
	TCP_RC_POS_MID,
	10, "sii"
};
tcp_rc_pos_t    msg_rc_pos;

#define I32TOBUF(i_, b_, s_) {b_[(s_)] = ((i_)>>24)&0xff; b_[(s_)+1] = ((i_)>>16)&0xff; b_[(s_)+2] = ((i_)>>8)&0xff; b_[(s_)+3] = ((i_)>>0)&0xff; }
#define I16TOBUF(i_, b_, s_) {b_[(s_)] = ((i_)>>8)&0xff; b_[(s_)+1] = ((i_)>>0)&0xff; }

void tcp_send_lidar(lidar_scan_t* p_lid)
{
	const int size = 13+180*2;
	uint8_t buf[size];
	buf[0] = TCP_RC_LIDAR_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;

	int r_ang = p_lid->robot_pos.ang>>16;
	int r_x = p_lid->robot_pos.x;
	int r_y = p_lid->robot_pos.y;

	I16TOBUF(r_ang, buf, 3);
	I32TOBUF(r_x, buf, 5);
	I32TOBUF(r_y, buf, 9);

	for(int i=0; i<360; i+=2)
	{
		int x = 0;
		int y = 0;
		// to save space, send every other sample; if the first is invalid, the second will be ok.
		// Also, to save space, send offsets to the middle.
		// Convert measurements to 4cm/unit so that int8 is enough. range=508cm. Saturate measurements over that.
		if(p_lid->scan[i].valid)
		{
			x = p_lid->scan[i].x - r_x;
			y = p_lid->scan[i].y - r_y;
		}
		else if(p_lid->scan[i+1].valid)
		{
			x = p_lid->scan[i+1].x;
			y = p_lid->scan[i+1].y;
		}
		// else send 0,0 to denote invalid point.

		x/=40;
		y/=40;

		if(x>127) x = 127; else if(x<-128) x=-128;
		if(y>127) y = 127; else if(y<-128) y=-128;

		// i goes += 2, which is handy here:
		buf[13+i] = (int8_t)x;
		buf[13+i+1] = (int8_t)y;

	}

	tcp_send(buf, size);

}

void tcp_send_hwdbg(int32_t* dbg)
{
	const int size = 3+10*4;
	uint8_t buf[size];
	buf[0] = TCP_RC_DBG_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;

	for(int i=0; i<10; i++)
	{
		I32TOBUF(dbg[i], buf, 3+i*4);
	}
	tcp_send(buf, size);
}

void tcp_send_sonar(sonar_scan_t* p_son)
{
	const int size = 3+1+3*2*4;
	uint8_t buf[size];
	buf[0] = TCP_RC_SONAR_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;
	buf[3] = p_son->scan[2].valid<<2 | p_son->scan[1].valid<<1 | p_son->scan[0].valid;

	for(int i=0; i<3; i++)
	{
		I32TOBUF(p_son->scan[i].x, buf, 4+i*8);
		I32TOBUF(p_son->scan[i].y, buf, 8+i*8);
	}
	tcp_send(buf, size);
}

int tcp_send_msg(tcp_message_t* msg_type, void* msg)
{
	static uint8_t sendbuf[65536];
	sendbuf[0] = msg_type->mid;
	sendbuf[1] = (msg_type->size>>8)&0xff;
	sendbuf[2] = msg_type->size&0xff;

	uint8_t* p_dest = sendbuf+3;
	uint8_t* p_src = msg;

	for(int field=0;;field++)
	{
		switch(msg_type->types[field])
		{
			case 'b':
			case 'B':
				*(p_dest++) = *(p_src++);
			break;

			case 's':
			case 'S':
			{
				*(p_dest++) = (((*((uint16_t*)p_src)) )>>8)&0xff;
				*(p_dest++) = (((*((uint16_t*)p_src)) )>>0)&0xff;
				p_src+=2;
			}
			break;

			case 'i':
			case 'I':
			{
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>24)&0xff;
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>16)&0xff;
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>8)&0xff;
				*(p_dest++) = (((*((uint32_t*)p_src)) )>>0)&0xff;
				p_src+=4;
			}
			break;

			case 'l':
			case 'L':
			{
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>56)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>48)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>40)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>32)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>24)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>16)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>8)&0xff;
				*(p_dest++) = (((*((uint64_t*)p_src)) )>>0)&0xff;
				p_src+=8;
			}
			break;

			case 0:
				goto PARSE_END;

			default:
				fprintf(stderr, "ERROR: parse type string has invalid character 0x%02x\n", msg_type->types[field]);
				return -2;
		}
	}

	PARSE_END: ;

	//printf("INFO: Sending tcp, size=%d\n", msg_type->size+3);
	tcp_send(sendbuf, msg_type->size+3);

	return 0;
}

/*
Return value:
< 0: Error.
0: Message was not parsed (not fully received yet)
>0: Message ID of the parsed message.
*/

int tcp_parser(int sock)
{
	int ret;
	static int state = 0; // Num of bytes read
	static struct __attribute__ ((packed)) { uint8_t mid; uint8_t size_msb; uint8_t size_lsb;} header; // Stored header of incoming message
	static int bytes_left = 0;
	static tcp_message_t* msg = 0; // Once message is recognized, pointer to message type struct.
	static int unrecog = 0;

	static uint8_t buf[65536];
	static uint8_t* p_buf = buf;

	if(state < 3)
	{
		msg = 0;
		unrecog = 0;
		ret = read(sock, (uint8_t*)&(header.mid) + state, 3-state);
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: TCP stream read error %d (%s)\n", errno, strerror(errno));
			state = 0;
			return -11;
		}
		else if(ret == 0)
		{
			fprintf(stderr, "INFO: Client closed connection\n");
			state = 0;
			return -10;
		}
		else
			state += ret;
	}

	if(state >= 3)
	{
		if(msg == 0 && unrecog == 0)
		{		
			for(int i=0; i<NUM_CR_MSGS; i++)
			{
				if(CR_MSGS[i]->mid == header.mid)
				{
					msg = CR_MSGS[i];
					break;
				}
			}

			int size_from_header = ((int)header.size_msb<<8) | (int)header.size_lsb;
			if(!msg)
			{
				fprintf(stderr, "WARN: Ignoring unrecognized message with msgid 0x%02x\n", header.mid);
				unrecog = 1;
			}
			else if(size_from_header != msg->size)
			{
				fprintf(stderr, "WARN: Ignoring message with msgid 0x%02x because of size mismatch (got:%u, expected:%u)\n",
					header.mid, size_from_header, msg->size);
				unrecog = 1;
			}
			bytes_left = size_from_header;
			p_buf = buf;
		}

		// Read as much data as the size field shows.
		if(bytes_left)
		{
			ret = read(sock, p_buf, bytes_left);
			if(ret < 0)
			{
				fprintf(stderr, "ERROR: TCP stream read error %d (%s)\n", errno, strerror(errno));
			}
			else if(ret == 0)
			{
				fprintf(stderr, "ERROR: TCP stream read() returned 0 bytes even when it shoudln't.\n");
			}
			else
			{
				bytes_left -= ret;
				p_buf += ret;
			}
		}

		if(bytes_left < 0)
		{
			fprintf(stderr, "ERROR: bytes_left < 0\n");
			bytes_left = 0;
		}

		if(bytes_left == 0)
		{
			state = 0;

			if(!unrecog)
			{
				// Parse the message
				uint8_t* p_src = buf;
				void* p_dest = msg->p_data;

				if(p_dest == 0)
				{
					fprintf(stderr, "ERROR: message parsing destination pointer NULL\n");
					return -1;
				}

				for(int field=0;;field++)
				{
					switch(msg->types[field])
					{
						case 'b':
						case 'B':
							*((uint8_t*)p_dest) = *(p_src++);
							p_dest = ((uint8_t*)p_dest) + 1;
						break;

						case 's':
						case 'S':
						{
							uint16_t tmp  = ((uint16_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint16_t*)p_dest) = tmp;
							p_dest = ((uint16_t*)p_dest) + 1;
						}
						break;

						case 'i':
						case 'I':
						{
							uint32_t tmp  = ((uint32_t)(*(p_src++))<<24);
								 tmp |= ((uint32_t)(*(p_src++))<<16);
								 tmp |= ((uint32_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint32_t*)p_dest) = tmp;
							p_dest = ((uint32_t*)p_dest) + 1;
						}
						break;

						case 'l':
						case 'L':
						{
							uint64_t tmp  = ((uint64_t)(*(p_src++))<<56);
								 tmp |= ((uint64_t)(*(p_src++))<<48);
								 tmp |= ((uint64_t)(*(p_src++))<<40);
								 tmp |= ((uint64_t)(*(p_src++))<<32);
								 tmp |= ((uint64_t)(*(p_src++))<<24);
								 tmp |= ((uint64_t)(*(p_src++))<<16);
								 tmp |= ((uint64_t)(*(p_src++))<<8);
								 tmp |=  *(p_src++);
							*((uint64_t*)p_dest) = tmp;
							p_dest = ((uint64_t*)p_dest) + 1;
						}
						break;


						case 0:
							goto PARSE_END;

						default:
							fprintf(stderr, "ERROR: parse type string has invalid character 0x%02x\n", msg->types[field]);
							return -2;

					}
				}

				PARSE_END: ;
				return header.mid;
			}
		}

	}

	return 0;
}

