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

*/

#ifndef PULUTOF_H
#define PULUTOF_H

#include "datatypes.h" // for pos_t


#define PULUTOF_STATUS_OVERFLOW  253
#define PULUTOF_STATUS_MULTIPLE  254
#define PULUTOF_STATUS_AVAILABLE 255
// 0..250: suggested sleep interval in 1ms units.


#define TOF_XS 160
#define TOF_YS 60

/*
	PORTABILITY WARNING:
	Assuming little endian on both sides. Developed for Raspi <-> Cortex M7.
*/
typedef struct __attribute__((packed))
{
	uint32_t header;
	uint8_t status; // Only read this and deassert chip select for polling the status
	uint8_t dummy1;
	uint8_t dummy2;
	uint8_t sensor_idx;

	pos_t robot_pos; // Robot pose during the acquisition

	uint16_t depth[TOF_XS*TOF_YS];
	uint8_t  ampl[TOF_XS*TOF_YS];
	uint8_t  ambient[TOF_XS*TOF_YS];

	uint16_t timestamps[24]; // 0.1ms unit timestamps of various steps for analyzing the timing of low-level processing

} pulutof_frame_t;



#endif
