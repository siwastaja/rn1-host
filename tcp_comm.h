#ifndef TCP_COMM_H
#define TCP_COMM_H

#include <stdint.h>

extern int tcp_listener_sock;
extern int tcp_client_sock; // One client at the time is allowed.

// Ring buffer implemented using overflowing uint16_t counters.
#define TCP_RX_BUF_LEN 65536
extern uint16_t tcp_rx_ring_wr;
extern uint16_t tcp_rx_ring_rd;
extern uint8_t tcp_buf[TCP_RX_BUF_LEN];

int init_tcp_comm();
int handle_tcp_client();
int handle_tcp_listener();




#endif
