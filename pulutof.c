/*
	PULUROBOT RN1-HOST Computer-on-RobotBoard main software

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.



	Driver for SPI-connected PULUTOF 3D Time-of-Flight add-on

	For Raspberry Pi 3, make sure that:
	dtparam=spi=on     is in /boot/config.txt uncommented
	/dev/spidev0.0 should exist

	Also:
	/boot/cmdline.txt:  spidev.bufsiz=65536
	This often defaults to 4096, which is ridiculously too small.
	(65535 is hardware maximum for STM32 DMA transfer, so bigger transfers
	couldn't be easily utilized, anyway.)

*/

#define _BSD_SOURCE  // glibc backwards incompatibility workaround to bring usleep back.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <unistd.h>

#include "pulutof.h"

#define PULUTOF_SPI_DEVICE "/dev/spidev0.0"


static int spi_fd;

static const unsigned char spi_mode = SPI_MODE_0;
static const unsigned char spi_bits_per_word = 8;
static const unsigned int spi_speed = 25000000; // Hz

static int init_spi()
{
	spi_fd = open(PULUTOF_SPI_DEVICE, O_RDWR);

	if(spi_fd < 0)
	{
		printf("ERROR: Opening PULUTOF SPI device %s failed: %d (%s).\n", PULUTOF_SPI_DEVICE, errno, strerror(errno));
		return -1;
	}

	/*
		SPI_MODE_0 CPOL = 0, CPHA = 0, Clock idle low, data is clocked in on rising edge, output data (change) on falling edge
		SPI_MODE_1 CPOL = 0, CPHA = 1, Clock idle low, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_2 CPOL = 1, CPHA = 0, Clock idle high, data is clocked in on falling edge, output data (change) on rising edge
		SPI_MODE_3 CPOL = 1, CPHA = 1, Clock idle high, data is clocked in on rising, edge output data (change) on falling edge
	*/

	/*
		Several code examples, (for example, http://www.raspberry-projects.com/pi/programming-in-c/spi/using-the-spi-interface),
		have totally misunderstood what SPI_IOC_WR_* and SPI_IOC_RD_* mean. They are not different settings for SPI RX/TX respectively
		(that wouldn't make any sense: SPI by definition has synchronous RX and TX so clearly the settings are always the same). Instead,
		SPI_IOC_WR_* and SPI_IOC_RD_* write and read the driver settings, respectively, as explained in spidev documentation:
		https://www.kernel.org/doc/Documentation/spi/spidev

		Here, we just set what we need.
	*/

	if(ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_MODE failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_BITS_PER_WORD failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	if(ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) < 0)
	{
		printf("ERROR: Opening PULUTOF SPI devide: ioctl SPI_IOC_WR_MAX_SPEED_HZ failed: %d (%s).\n", errno, strerror(errno));
		return -2;
	}

	return 0;
}


static int deinit_spi()
{
	if(close(spi_fd) < 0)
	{
		printf("WARNING: Closing PULUTOF SPI devide failed: %d (%s).\n", errno, strerror(errno));
		return -1;
	}

	return 0;
}

volatile int dbg_id = 0;

void pulutof_decr_dbg()
{
	if(dbg_id) dbg_id--;
	printf("PULUTOF dbg_id=%d\n", dbg_id);
}

void pulutof_incr_dbg()
{
	dbg_id++;
	printf("PULUTOF dbg_id=%d\n", dbg_id);
}

#define PULUTOF_RINGBUF_LEN 16
pulutof_frame_t pulutof_ringbuf[PULUTOF_RINGBUF_LEN];
int pulutof_ringbuf_wr = 0;
int pulutof_ringbuf_rd = 0;

pulutof_frame_t* get_pulutof_frame()
{
	if(pulutof_ringbuf_wr == pulutof_ringbuf_rd)
	{
		return 0;
	}
	
	pulutof_frame_t* ret = &pulutof_ringbuf[pulutof_ringbuf_rd];
	pulutof_ringbuf_rd++; if(pulutof_ringbuf_rd >= PULUTOF_RINGBUF_LEN) pulutof_ringbuf_rd = 0;
	return ret;
}

/*
	As you should know, SPI is a "show yours, I'll show mine" protocol. As a master, we'll only send something, and get something back.
	Unfortunately, Raspberry Pi can only be a master, in SPI terms.
	However, the PULUTOF in kind of a "real" master, because it decides when to image the frames, and will do it regardless of what we are
	doing on the Raspberry - the data is anyway needed for low-level obstacle avoidance, completely independent of the Raspi.

	poll_availability commands a small (one-byte) transfer, which will only return the first status byte. If we are happy with the status (i.e., there is
	data available), we can go on with the full transfer. This data further gives insight how long to wait before next polling is needed.

	When poll_availability informs there is a frame available, read_frame can be called - it assumes the data is indeed available and performs a full
	frame read (if the data is not available, you'll hit an old frame, or possibly a corrupted combination of two frames).
*/


static uint8_t txbuf[65536];


static int poll_availability()
{
	txbuf[4] = dbg_id&0xff;	
	struct spi_ioc_transfer xfer;
	struct response { uint32_t header; uint8_t status;} response;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	//xfer.tx_buf left at 0 - documented spidev feature to send out zeroes - we don't have anything to send, just want to get what the sensor wants to send us!
	xfer.tx_buf = txbuf;
	xfer.rx_buf = &response;
	xfer.len = sizeof response;
	xfer.cs_change = 0; // deassert chip select after the transfer

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}

	if(response.header != 0x11223344 || response.status == 0)
	{
		printf("ERROR: Illegal response in poll_availability: header=0x%08x  status=%d\n", response.header, response.status);
		return -1;
	}
	//printf("status=%d\n", response.status);
	return response.status;
}

static int read_frame()
{
	txbuf[4] = dbg_id&0xff;	
	struct spi_ioc_transfer xfer;
	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.

	//xfer.tx_buf left at 0 - documented spidev feature to send out zeroes - we don't have anything to send, just want to get what the sensor wants to send us!
	xfer.tx_buf = txbuf;
	xfer.rx_buf = &pulutof_ringbuf[pulutof_ringbuf_wr];
	xfer.len = sizeof(pulutof_frame_t);
	xfer.cs_change = 0; // deassert chip select after the transfer

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return -1;
	}

	printf("Frame read ok, timing:\n");
	for(int i=0; i<24; i++)
	{
		printf("%d:%.1f ", i, (float)pulutof_ringbuf[pulutof_ringbuf_wr].timestamps[i]/10.0);
	}
	printf("\n");
	printf("Time deltas:\n");
	for(int i=1; i<24; i++)
	{
		printf("%d->%d:%.1f ", i-1, i, (float)(pulutof_ringbuf[pulutof_ringbuf_wr].timestamps[i]-pulutof_ringbuf[pulutof_ringbuf_wr].timestamps[i-1])/10.0);
	}
	printf("\n");

	int ret = pulutof_ringbuf[pulutof_ringbuf_wr].status;

	pulutof_ringbuf_wr++; if(pulutof_ringbuf_wr >= PULUTOF_RINGBUF_LEN) pulutof_ringbuf_wr = 0;

	return ret;
}

int running = 1;

void request_tof_quit()
{
	running = 0;
}

void pulutof_cal_offset()
{
	printf("Requesting offset calib\n");
	struct spi_ioc_transfer xfer;
	struct cmd { uint32_t header; uint32_t dummy;} cmd;

	cmd.header = 0xca0ff5e7;
	cmd.dummy = 0;

	memset(&xfer, 0, sizeof(xfer)); // unused fields need to be initialized zero.
	xfer.tx_buf = &cmd;
	xfer.rx_buf = NULL;
	xfer.len = sizeof cmd;
	xfer.cs_change = 0; // deassert chip select after the transfer

	if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &xfer) < 0)
	{
		printf("ERROR: spi ioctl transfer operation failed: %d (%s)\n", errno, strerror(errno));
		return;
	}

}


void* pulutof_poll_thread()
{
	init_spi();
	while(running)
	{
		int next = pulutof_ringbuf_wr+1; if(next >= PULUTOF_RINGBUF_LEN) next = 0;
		if(next == pulutof_ringbuf_rd)
		{
			printf("WARNING: PULUTOF ringbuf overflow prevented, ignoring images...\n");
			usleep(250000);
			continue;
		}


		int avail = poll_availability();

		if(avail < 0)
		{
			break;
		}

		if(avail < 250)
		{
	//		printf("Sleeping %d ms\n", avail);
			usleep(1000*avail);
			continue;
		}

		read_frame();



		usleep(1000);
	}
	deinit_spi();

	return NULL;
}

