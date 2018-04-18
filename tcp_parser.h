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



*/

#ifndef TCP_PARSER_H
#define TCP_PARSER_H

#include "datatypes.h"
#include "routing.h"

typedef struct
{
	// Where to write when receiving. This field is ignored for tx. 
	// Remember to use packed structs
	void* p_data; 
	// Message ID
	uint8_t mid;
	// Number of bytes of data expected / sent
	int size;
	// Zero-terminated string: Interpretation of the bytes:
	char types[32]; 
	/*
		'b' int8_t
		'B' uint8_t
		's' int16_t
		'S' uint16_t
		'i' int32_t
		'I' uint32_t
		'l' int64_t
		'L' uint64_t
	*/
	int ret;
} tcp_message_t;


#define TCP_CR_DEST_MID    55
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
	int8_t backmode;
} tcp_cr_dest_t;

extern tcp_cr_dest_t   msg_cr_dest;

#define TCP_CR_ROUTE_MID    56
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
	int8_t dummy;
} tcp_cr_route_t;

extern tcp_cr_route_t   msg_cr_route;


#define TCP_CR_CHARGE_MID    57
typedef struct __attribute__ ((packed))
{
	uint8_t params;
} tcp_cr_charge_t;

extern tcp_cr_charge_t   msg_cr_charge;

#define TCP_CR_MODE_MID    58
typedef struct __attribute__ ((packed))
{
	uint8_t mode;
} tcp_cr_mode_t;

extern tcp_cr_mode_t   msg_cr_mode;

#define TCP_CR_MANU_MID    59
typedef struct __attribute__ ((packed))
{
	uint8_t op;
} tcp_cr_manu_t;

extern tcp_cr_manu_t   msg_cr_manu;

#define TCP_CR_ADDCONSTRAINT_MID    60
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
} tcp_cr_addconstraint_t;

extern tcp_cr_addconstraint_t   msg_cr_addconstraint;

#define TCP_CR_REMCONSTRAINT_MID    61
typedef struct __attribute__ ((packed))
{
	int32_t x;
	int32_t y;
} tcp_cr_remconstraint_t;

extern tcp_cr_remconstraint_t   msg_cr_remconstraint;

#define TCP_CR_MAINTENANCE_MID    62
typedef struct __attribute__ ((packed))
{
	int32_t magic;
	int32_t retval;
} tcp_cr_maintenance_t;

extern tcp_cr_maintenance_t   msg_cr_maintenance;

#define TCP_CR_SPEEDLIM_MID    63
typedef struct __attribute__ ((packed))
{
	uint8_t speedlim_linear_fwd;  // right now, sets both linear (fwd and back) and angular speedlimit. 1..100. 0 = use default limit. Note that actual speedlimit may always be lower due to nearby obstacles.
	uint8_t speedlim_linear_back; // for future use, not implemented yet 
	uint8_t speedlim_angular; // for future use, not implemented yet
	uint8_t accellim_linear;  // for future use, not implemented yet
	uint8_t accellim_angular; // for future use, not implemented yet
} tcp_cr_speedlim_t;

extern tcp_cr_speedlim_t   msg_cr_speedlim;


#define TCP_CR_STATEVECT_MID        64


#define TCP_RC_POS_MID    130
typedef struct __attribute__ ((packed))
{
	int16_t ang;
	int32_t x;
	int32_t y;
	uint8_t  cmd_state; // Message ID of the command/job the robot is currently taking (for example, TCP_CR_ROUTE_MID)
} tcp_rc_pos_t;

extern tcp_message_t   msgmeta_rc_pos;
extern tcp_rc_pos_t    msg_rc_pos;


#define TCP_RC_MOVEMENT_STATUS_SUCCESS 0
#define TCP_RC_MOVEMENT_STATUS_STOPPED 1
#define TCP_RC_MOVEMENT_STATUS_STOPPED_BY_FEEDBACK_MODULE 1
#define TCP_RC_MOVEMENT_STATUS_MID  143
typedef struct __attribute__ ((packed))
{
	int16_t start_ang;
	int32_t start_x;
	int32_t start_y;

	int32_t requested_x;
	int32_t requested_y;
	int8_t requested_backmode;

	int16_t cur_ang;
	int32_t cur_x;
	int32_t cur_y;

	uint8_t status;
	uint32_t obstacle_flags;
} tcp_rc_movement_status_t;
extern tcp_message_t   msgmeta_rc_movement_status;
extern tcp_rc_movement_status_t    msg_rc_movement_status;


#define TCP_RC_ROUTE_STATUS_SUCCESS 0
#define TCP_RC_ROUTE_STATUS_NOTFOUND 1
#define TCP_RC_ROUTE_STATUS_UNDEFINED 2
#define TCP_RC_ROUTE_STATUS_MID  144
typedef struct __attribute__ ((packed))
{
	int16_t start_ang;
	int32_t start_x;
	int32_t start_y;

	int32_t requested_x;
	int32_t requested_y;

	int16_t cur_ang;
	int32_t cur_x;
	int32_t cur_y;

	uint8_t status;
	int16_t num_reroutes;
} tcp_rc_route_status_t;
extern tcp_message_t   msgmeta_rc_route_status;
extern tcp_rc_route_status_t    msg_rc_route_status;


#define TCP_RC_LIDAR_LOWRES_MID     131
#define TCP_RC_DBG_MID              132
#define TCP_RC_SONAR_MID            133
#define TCP_RC_BATTERY_MID          134
#define TCP_RC_ROUTEINFO_MID        135
#define TCP_RC_SYNCREQ_MID          136
#define TCP_RC_DBGPOINT_MID         137
#define TCP_RC_HMAP_MID             138
#define TCP_RC_INFOSTATE_MID        139
#define TCP_RC_ROBOTINFO_MID        140
#define TCP_RC_LIDAR_HIGHRES_MID    141
#define TCP_RC_PICTURE_MID	    142

#define TCP_RC_STATEVECT_MID        145


int tcp_parser(int sock);

int tcp_send_msg(tcp_message_t* msg_type, void* msg);

void tcp_send_lidar_lowres(lidar_scan_t* p_lid);
void tcp_send_lidar_highres(lidar_scan_t* p_lid);
void tcp_send_hwdbg(int32_t* dbg);
void tcp_send_sonar(sonar_point_t* p_son);
void tcp_send_battery();
void tcp_send_route(int32_t first_x, int32_t first_y, route_unit_t **route);
void tcp_send_sync_request();
void tcp_send_dbgpoint(int x, int y, uint8_t r, uint8_t g, uint8_t b, int persistence);
void tcp_send_hmap(int xsamps, int ysamps, int32_t ang, int xorig_mm, int yorig_mm, int unit_size_mm, int8_t *hmap);
void tcp_send_info_state(info_state_t state);
void tcp_send_robot_info();
void tcp_send_picture(int16_t id, uint8_t bytes_per_pixel, int xs, int ys, uint8_t *pict);
void tcp_send_statevect();


#endif
