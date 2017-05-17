#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>

int tcp_listener_sock;
int tcp_client_sock = -1; // One client at the time is allowed.

int build_socket(uint16_t port)
{
	int sock;
	struct sockaddr_in name;

	sock = socket(PF_INET, SOCK_STREAM, 0);
	if (sock < 0)
	{
		perror ("socket");
		exit(EXIT_FAILURE);
	}

	name.sin_family = AF_INET;
	name.sin_port = htons (port);
	name.sin_addr.s_addr = htonl (INADDR_ANY);
	if (bind(sock, (struct sockaddr *) &name, sizeof (name)) < 0)
	{
		perror("bind");
		exit(EXIT_FAILURE);
	}

	return sock;
}

int init_tcp_comm()
{
	/* Create the socket and set it up to accept connections. */
	tcp_listener_sock = build_socket(port);
	if(listen(tcp_listener_sock, 1) < 0)
	{
		perror ("listen");
		exit(EXIT_FAILURE);
	}

	return 0;
}

// Call this when you have data (connection request) in tcp_listener_sock input buffer, for example, after using select()
int handle_tcp_listener()
{
	int new_fd;
	struct sockaddr_in clientname;
	size_t size = sizeof(clientname);
	new_fd = accept(tcp_listener_socket, (struct sockaddr *) &clientname, &size);
	if(new_fd < 0)
	{
		fprintf(stderr, "ERROR: accepting tcp connection request failed.\n");
	}

	if(tcp_client_sock >= 0)
	{
		// We already have a client; disconnect it.
		close(tcp_client_sock);
	}

	tcp_client_sock = new_fd;
	printf("INFO: connection accepted, client %08x, port %hd.\n", clientname.sin_addr.s_addr,
	        ntohs(clientname.sin_port));
}

// Ring buffer implemented using overflowing uint16_t counters.
#define TCP_RX_BUF_LEN 65536
uint16_t tcp_rx_ring_wr;
uint16_t tcp_rx_ring_rd;
uint8_t tcp_buf[TCP_RX_BUF_LEN];

// Call this when you have data in tcp_client_sock input buffer, for example, after using select()
int handle_tcp_client()
{
	while(1)
	{
		int space_left = TCP_RX_BUF_LEN - (int)tcp_rx_ring_wr;
		int n_read = read(tcp_client_sock, tcp_buf, space_left);
		if(n_read < 0)
		{
			fprintf(stderr, "TCP client socket read error %d (%s)\n", errno, strerror(errno));
			return 1;
		}
		if(n_read == 0)
		{
			return 0;
		}
		tcp_rx_ring_wr += n_read; // Overflow is desired, implementing ring buffer.
		usleep(10);
	}
}
