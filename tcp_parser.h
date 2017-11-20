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


#define TCP_RC_POS_MID    130
typedef struct __attribute__ ((packed))
{
	int16_t ang;
	int32_t x;
	int32_t y;
} tcp_rc_pos_t;

extern tcp_message_t   msgmeta_rc_pos;
extern tcp_rc_pos_t    msg_rc_pos;

#define TCP_RC_LIDAR_LOWRES_MID     131
#define TCP_RC_DBG_MID       132
#define TCP_RC_SONAR_MID     133
#define TCP_RC_BATTERY_MID   134
#define TCP_RC_ROUTEINFO_MID 135
#define TCP_RC_SYNCREQ_MID   136
#define TCP_RC_DBGPOINT_MID  137
#define TCP_RC_HMAP_MID      138
#define TCP_RC_INFOSTATE_MID 139
#define TCP_RC_ROBOTINFO_MID 140
#define TCP_RC_LIDAR_HIGHRES_MID     141

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


#endif
