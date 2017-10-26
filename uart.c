#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "hwdata.h"
#include "../rn1-brain/comm.h" // macros for 7-bit data management

#define MAXBUF 2048

static int set_uart_attribs(int fd, int speed)
{
	struct termios tty;
	memset(&tty, 0, sizeof(tty));
	if(tcgetattr(fd, &tty) != 0)
	{
		printf("error %d from tcgetattr\n", errno);
		return -1;
	}

	cfmakeraw(&tty);
	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;

	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~(PARENB | PARODD);
	tty.c_cflag &= ~CSTOPB;

	// nonblocking
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 0;

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	if(tcsetattr(fd, TCSANOW, &tty) != 0)
	{
		printf("error %d from tcsetattr\n", errno);
		return -1;
	}
	return 0;
}

int uart;

const char serial_dev[] = "/dev/serial0";
//const char serial_dev[] = "/dev/ttyUSB0";

int init_uart()
{
	uart = open(serial_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(uart < 0)
	{
		printf("error %d opening %s: %s\n", errno, serial_dev, strerror(errno));
		return 1;
	}

	set_uart_attribs(uart, B115200);

	return 0;

}

int send_uart(uint8_t* buf, int len)
{
	int timeout = 100;
	while(len > 0)
	{
		int ret = write(uart, buf, len);
		if(ret == 0)
		{
			usleep(1000);
			timeout--;
			if(timeout == 0)
			{
				fprintf(stderr, "uart write error: timeout, write() returns 0 for 100 ms\n");
				return 1;
			}
		}
		else if(ret == -1)
		{
			fprintf(stderr, "uart write error %d: %s\n", errno, strerror(errno));
			return 2;
		}
		else
		{
			len -= ret;
			buf += ret;
		}
	}

	return 0;
}

#define UART_RX_BUF_SIZE 65536
uint8_t uart_rx_buf[UART_RX_BUF_SIZE];
int uart_rx_wr;
int uart_rx_rd;

#define SECOND_FETCH_AMOUNT 1000

/*
UART packet structure

1 byte: Packet type header
2 bytes: Packet payload size (little endian)
Payload 1 to 8192 bytes (arbitrary limitation right now.)
1 byte: CRC8 over the payload


resynchronization sequence:
0xaa (header) 0x08 0x00 (payload size) 0xff,0xff,0xff,0xff,0xff,0xff,0x12,0xab (payload)  + CRC8 (should be 0xd6)
*/


typedef enum
{
	S_UART_RESYNC	= 0,
	S_UART_HEADER	= 1,
	S_UART_PAYLOAD	= 2,
	S_UART_CHECKSUM = 3
} uart_state_t;

uart_state_t cur_uart_state;

uint8_t uart_payload[8192];


#define CRC_INITIAL_REMAINDER 0x00
#define CRC_POLYNOMIAL 0x07 // As per CRC-8-CCITT

#define CALC_CRC(remainder) \
	for(int crc__bit = 8; crc__bit > 0; --crc__bit) \
	{ \
		if((remainder) & 0b10000000) \
		{ \
			(remainder) = ((remainder) << 1) ^ CRC_POLYNOMIAL; \
		} \
		else \
		{ \
			(remainder) = ((remainder) << 1); \
		} \
	}

// Call this when there is data in the uart read buffer (using select, for example)
void handle_uart()
{
	static uint8_t header[3];
	static int header_left;
	static int msg_id, msg_size, payload_left;
	static uint8_t checksum_accum;

	switch(cur_uart_state)
	{
		case S_UART_RESYNC:
		{
			static int resync_cnt = 0;
			static const uint8_t expected_resync[12] = {0xaa,0x08,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,0x12,0xab, 0xd6};

			uint8_t byte;
			int ret = read(uart, &byte, 1);
			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in RESYNC, errno=%d\n", errno);
				resync_cnt = 0;
			}
			else if(ret == 1)
			{
				//printf("0x%02x  cnt=%d\n", byte, resync_cnt);
				if(byte == expected_resync[resync_cnt])
				{
					resync_cnt++;
					if(resync_cnt == 12)
					{
						resync_cnt = 0;
						cur_uart_state = S_UART_HEADER;
						header_left = 3;
						printf("INFO: UART RESYNC OK.\n");
						break;
					}
				}
				else
				{
					resync_cnt = 0;
				}
			}
			else
			{
				printf("ERROR: read() (uart) returned %d in RESYNC\n", ret);
			}
		}
		break;

		case S_UART_HEADER:
		{
			int ret = read(uart, &header[3-header_left], header_left);
			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in S_UART_HEADER, errno=%d\n", errno);
				cur_uart_state = S_UART_RESYNC;
				break;
			}

			header_left -= ret;

			if(header_left == 0)
			{
				msg_id = header[0];
				msg_size = ((int)header[2])<<8 | ((int)header[1]);

				if(msg_size > 8192)
				{
					printf("ERROR: Invalid msg_size (uart)\n");
					cur_uart_state = S_UART_RESYNC;
					break;
				}
				payload_left = msg_size;
				checksum_accum = CRC_INITIAL_REMAINDER;
				cur_uart_state = S_UART_PAYLOAD;
				//printf("INFO: Got header, msgid=%d, len=%d\n", msg_id, msg_size);

			}
		}
		break;

		case S_UART_PAYLOAD:
		{
			int ret = read(uart, &uart_payload[msg_size-payload_left], payload_left);
			//printf("PAYLOAD: read() msg_size=%d, payload_left=%d, idx=%d, data = ", msg_size, payload_left, msg_size-payload_left);
			//for(int i = 0; i < ret; i++)
			//	printf("%02x ", uart_payload[msg_size-payload_left+i]);
			//printf("\n");
			

			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in S_UART_PAYLOAD, errno=%d\n", errno);
				cur_uart_state = S_UART_RESYNC;
				break;
			}

			for(int i = 0; i < ret; i++)
			{
				checksum_accum ^= uart_payload[msg_size-payload_left+i];
				CALC_CRC(checksum_accum);
			}

			//printf("INFO: READ %d BYTES OF PAYLOAD, %d left (1st: %d)\n", ret, payload_left, uart_payload[msg_size-payload_left]);

			payload_left -= ret;

			if(payload_left == 0)
			{
				cur_uart_state = S_UART_CHECKSUM;
			}

		}
		break;

		case S_UART_CHECKSUM:
		{
			uint8_t chk;
			int ret = read(uart, &chk, 1);
			if(ret < 0)
			{
				printf("ERROR: read() (uart) failed in S_UART_CHECKSUM, errno=%d\n", errno);
				cur_uart_state = S_UART_RESYNC;
				break;
			}

			if(ret == 1)
			{
				if(chk != checksum_accum)
				{
					printf("WARN: UART checksum mismatch (msgid=%d len=%d chk_received=%02x calculated=%02x), ignoring message & resyncing.\n", msg_id, msg_size, chk, checksum_accum);

					//for(int i = 0; i < msg_size; i++)
					//{
					//	printf("%02x ", uart_payload[i]);
					//	if(i%8==7) printf("\n");
					//}
					//printf("\n");
					cur_uart_state = S_UART_RESYNC;
					break;
				}

				//printf("INFO: Parsing UART MESSAGE (msgid=%d len=%d chksum=%02x)\n", msg_id, msg_size, chk);
				parse_uart_msg(uart_payload, msg_id, msg_size);
				cur_uart_state = S_UART_HEADER;
				header_left = 3;

			}
		}
		break;

		default:
		break;

	}
}

