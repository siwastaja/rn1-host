#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "datatypes.h"
#include "tcp_comm.h"
#include "tcp_parser.h"
#include "utlist.h"
#include "routing.h"

// Client->Robot messages

tcp_cr_dest_t msg_cr_dest;
tcp_message_t msgmeta_cr_dest =
{
	&msg_cr_dest,
	TCP_CR_DEST_MID,
	9, "iib"
};

tcp_cr_route_t msg_cr_route;
tcp_message_t msgmeta_cr_route =
{
	&msg_cr_route,
	TCP_CR_ROUTE_MID,
	9, "iib"
};

tcp_cr_charge_t msg_cr_charge;
tcp_message_t msgmeta_cr_charge =
{
	&msg_cr_charge,
	TCP_CR_CHARGE_MID,
	1, "b"
};

tcp_cr_mode_t msg_cr_mode;
tcp_message_t msgmeta_cr_mode =
{
	&msg_cr_mode,
	TCP_CR_MODE_MID,
	1, "b"
};

tcp_cr_manu_t msg_cr_manu;
tcp_message_t msgmeta_cr_manu =
{
	&msg_cr_manu,
	TCP_CR_MANU_MID,
	1, "b"
};

tcp_cr_addconstraint_t msg_cr_addconstraint;
tcp_message_t msgmeta_cr_addconstraint =
{
	&msg_cr_addconstraint,
	TCP_CR_ADDCONSTRAINT_MID,
	8, "ii"
};

tcp_cr_remconstraint_t msg_cr_remconstraint;
tcp_message_t msgmeta_cr_remconstraint =
{
	&msg_cr_remconstraint,
	TCP_CR_REMCONSTRAINT_MID,
	8, "ii"
};

#define NUM_CR_MSGS 5
tcp_message_t* CR_MSGS[NUM_CR_MSGS] =
{
	&msgmeta_cr_dest,
	&msgmeta_cr_route,
	&msgmeta_cr_charge,
	&msgmeta_cr_mode,
	&msgmeta_cr_manu
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

void tcp_send_robot_info()
{
	const int size = 3+8;
	uint8_t buf[size];
	buf[0] = TCP_RC_ROBOTINFO_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;

	extern float main_robot_xs;
	extern float main_robot_ys;
	extern float main_robot_middle_to_lidar;
	int16_t rx = main_robot_xs;
	int16_t ry = main_robot_ys;
	int16_t lid_xoffs = -1.0*main_robot_middle_to_lidar;
	int16_t lid_yoffs = 0;

	I16TOBUF(rx, buf, 3);
	I16TOBUF(ry, buf, 5);
	I16TOBUF(lid_xoffs, buf, 7);
	I16TOBUF(lid_yoffs, buf, 9);

	tcp_send(buf, size);
}


void tcp_send_info_state(info_state_t state)
{
	static info_state_t prev = INFO_STATE_UNDEF;

	if(state != prev)
	{
		prev = state;
		const int size = 3+1;
		uint8_t buf[size];
		buf[0] = TCP_RC_INFOSTATE_MID;
		buf[1] = ((size-3)>>8)&0xff;
		buf[2] = (size-3)&0xff;
		buf[3] = (int8_t)state;
		tcp_send(buf, size);
	}
}

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

void tcp_send_sync_request()
{
	const int size = 3+1;
	uint8_t buf[size];
	buf[0] = TCP_RC_SYNCREQ_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;
	buf[3] = 0;

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
	buf[3] = (p_son->scan[2].valid<<2) | (p_son->scan[1].valid<<1) | (p_son->scan[0].valid);

	for(int i=0; i<3; i++)
	{
		I32TOBUF(p_son->scan[i].x, buf, 4+i*8);
		I32TOBUF(p_son->scan[i].y, buf, 8+i*8);
	}
	tcp_send(buf, size);
}

void tcp_send_hmap(int xsamps, int ysamps, int32_t ang, int xorig_mm, int yorig_mm, int unit_size_mm, int8_t *hmap)
{
	if(xsamps < 1 || xsamps > 60 || ysamps < 1 || ysamps > 60 || unit_size_mm < 2 || unit_size_mm > 200 || !hmap)
	{
		printf("ERR: tcp_send_hmap() argument sanity check fail\n");
		return;
	}

	int size = 3 + 1+1+4+4+2+1+xsamps*ysamps;
	uint8_t *buf = malloc(size);
	buf[0] = TCP_RC_HMAP_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;

	buf[3] = xsamps;
	buf[4] = ysamps;
	I16TOBUF((ang>>16), buf, 5);
	I32TOBUF(xorig_mm, buf, 7);
	I32TOBUF(yorig_mm, buf, 11);
	buf[15] = unit_size_mm;

	memcpy(&buf[16], (uint8_t*)hmap, xsamps*ysamps);

	tcp_send(buf, size);
	free(buf);
}


void tcp_send_battery()
{
	const int size = 7;
	uint8_t buf[size];
	buf[0] = TCP_RC_BATTERY_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;
	buf[3] = (pwr_status.charging?1:0) | (pwr_status.charged?2:0);
	buf[4] = (pwr_status.bat_mv>>8)&0xff;
	buf[5] = (pwr_status.bat_mv)&0xff;
	buf[6] = (pwr_status.bat_percentage)&0xff;

	tcp_send(buf, size);
}

void tcp_send_route(route_unit_t **route)
{
	uint8_t buf[2000];
	buf[0] = TCP_RC_ROUTEINFO_MID;

	int i = 3;
	route_unit_t *rt;
	DL_FOREACH(*route, rt)
	{
		if(i > 1999-9)
		{
			printf("WARNING: Route too long to be sent to the client. Ignoring the rest.\n");
			break;
		}

		int x_mm, y_mm;
		mm_from_unit_coords(rt->loc.x, rt->loc.y, &x_mm, &y_mm);					
		buf[i+0] = rt->backmode?1:0;
		I32TOBUF(x_mm, buf, i+1);
		I32TOBUF(y_mm, buf, i+5);
		i += 9;
	}

	buf[1] = ((i-3)>>8)&0xff;
	buf[2] = (i-3)&0xff;

	tcp_send(buf, i);
}

void tcp_send_dbgpoint(int x, int y, uint8_t r, uint8_t g, uint8_t b, int persistence)
{
	const int size = 3+4+4+3+1;
	uint8_t buf[size];
	buf[0] = TCP_RC_DBGPOINT_MID;
	buf[1] = ((size-3)>>8)&0xff;
	buf[2] = (size-3)&0xff;
	I32TOBUF(x, buf, 3);
	I32TOBUF(y, buf, 7);
	buf[11] = r;
	buf[12] = g;
	buf[13] = b;
	buf[14] = persistence&0xff;
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

	//printf("Sending tcp, size=%d\n", msg_type->size+3);
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
			fprintf(stderr, "Client closed connection\n");
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

