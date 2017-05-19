#ifndef UART_H
#define UART_H

extern int uart;

int init_uart();
int send_uart(uint8_t buf, int len);
void handle_uart();


void send_keepalive();
void move_to(int32_t x, int32_t y);



#endif
