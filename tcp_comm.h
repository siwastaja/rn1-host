#ifndef TCP_COMM_H
#define TCP_COMM_H

#include <stdint.h>

extern int tcp_listener_sock;
extern int tcp_client_sock; // One client at the time is allowed.

int init_tcp_comm();
int handle_tcp_client();
int handle_tcp_listener();
int tcp_send(uint8_t* buf, int len);




#endif
