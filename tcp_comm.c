#define _BSD_SOURCE // for usleep
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

#include "tcp_parser.h"

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
	tcp_listener_sock = build_socket(22222);
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
	new_fd = accept(tcp_listener_sock, (struct sockaddr *) &clientname, &size);
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
	printf("INFO: connection accepted, client %08x, port %u.\n", clientname.sin_addr.s_addr,
	        ntohs(clientname.sin_port));
	return 0;
}

// Call this when you have data in tcp_client_sock input buffer, for example, after using select()
int handle_tcp_client()
{
	int ret = tcp_parser(tcp_client_sock);
	if(ret == -10 || ret == -11)
	{
		printf("Info: closing TCP connection.\n");
		close(tcp_client_sock);
		tcp_client_sock = -1;
	}
	return ret;
}

int tcp_send(uint8_t* buf, int len)
{
	int timeout = 100;
	uint8_t* p_buf = buf;
	while(len)
	{
		printf("INFO: write %d  %d\n", tcp_client_sock, len);
		int ret = write(tcp_client_sock, p_buf, len);
		if(ret < 0)
		{
			fprintf(stderr, "ERROR: tcp_send(): socket write error %d (%s). Closing TCP connection.\n", errno, strerror(errno));
			close(tcp_client_sock);
			tcp_client_sock = -1;
			return -1;
		}
		else if(ret == 0)
		{
			printf("INFO: tcp_send(): write() wrote 0, closing TCP connection.\n");
			close(tcp_client_sock);
			tcp_client_sock = -1;
			return -2;
		}
		
		len -= ret;
		p_buf += ret;
		if(len)
		{
			printf("INFO: tcp_send(): write() didn't write everything, written=%d, left=%d\n", ret, len);
			usleep(100);
			timeout--;

			if(timeout == 0)
			{
				fprintf(stderr, "ERROR: tcp_send() timeouted because write() doesn't seem to do anything useful. Closing TCP connection.\n");
				close(tcp_client_sock);
				tcp_client_sock = -1;
				return -1;
			}
		}
	}

	return 0;
}
