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

#ifndef TCP_COMM_H
#define TCP_COMM_H

#include <stdint.h>

extern int tcp_listener_sock;
extern int tcp_client_sock; // One client at the time is allowed.

int init_tcp_comm();
int handle_tcp_client();
int handle_tcp_listener();
int tcp_send(uint8_t* buf, int len);
void tcp_comm_close();




#endif
