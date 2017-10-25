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

#if SSIZE_MAX > 1000
#define SECOND_FETCH_AMOUNT 1000
#else
#define SECOND_FETCH_AMOUNT SSIZE_MAX
#endif

/*
UART packet structure

1 byte: Packet type header
2 bytes: Packet payload size (little endian)
Payload 1 to 16384 bytes (arbitrary limitation right now.)
1 byte: CRC8 over the payload


resynchronization sequence:
0xaa (header) 0x08 0x00 (payload size) 0xff,0xff,0xff,0xff,0xff,0xff,0x12,0xab (payload)  + CRC8 (should be 0x83)
*/


typedef enum
{
	S_UART_RESYNC = 0,
} uart_state_t;

uart_state_t cur_uart_state;

// Call this when there is data in the uart read buffer (using select, for example)
void handle_uart()
{
	int amount = UART_RX_BUF_SIZE - uart_rx_wr;
	if(amount > SSIZE_MAX) amount = SSIZE_MAX; // requirement by read()
	int ret = read(uart, &uart_rx_fifo[uart_rx_wr], amount);
	if(ret < 0)
	{
		printf("read() (uart) failed");
		return;
	}

	uart_rx_wr += ret;

	if(uart_rx_wr > 65536)
	{
		printf("ERROR: uart_wr > 65536! Trying to recover.\n");
		uart_rx_wr = 0;
		return;
	}

	if(uart_rx_wr == 65536)
	{
		uart_rx_wr = 0;
		// We still might have something in the read buffer, try to fetch it:
		int ret2 = read(uart, &uart_rx_fifo[uart_rx_wr], SECOND_FETCH_AMOUNT);
		if(ret2 >= 0)
		{
			uart_rx_wr += ret2;
		}
	}

	// Process everything we have in the buffer
	while(uart_rx_rd != uart_rx_wr)
	{
		static int resync_cnt = 0;
		static const uint8_t expected_resync[12] = {0xaa,0x80,0x00, 0xff,0xff,0xff,0xff,0xff,0xff,0x12,0xab, 0x83};
		switch(cur_uart_state)
		{
			case S_UART_RESYNC:
			{
				if(uart_
			}
			break;
		}
	}


		parse_uart_msg(uart_rx_buf, rxloc);
}

