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

// Call this when there is data in the uart read buffer (using select, for example)
void handle_uart()
{
	static int rxloc = 0;
	static uint8_t uart_rx_buf[MAXBUF];

	uint8_t byte;

	// TODO: Write this to read as much as there is at once.
	if(read(uart, &byte, 1) < 0)
	{
		printf("read() (uart) failed");
		return;
	}

	if(rxloc > 1000)
		rxloc = 0;

	if(byte > 127)
	{
		parse_uart_msg(uart_rx_buf, rxloc);
		rxloc = 0;
	}

	uart_rx_buf[rxloc] = byte;
	rxloc++;
}

