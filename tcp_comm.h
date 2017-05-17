#ifndef TCP_COMM_H
#define TCP_COMM_H

extern int tcp_listener_sock;
extern int tcp_client_sock; // One client at the time is allowed.

int init_tcp_comm();


#endif
