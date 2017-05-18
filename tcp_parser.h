#ifndef TCP_PARSER_H
#define TCP_PARSER_H

typedef struct
{
	// Where to write / where to read from; 
	// if writing to NULL, message is dumped to stdout for debug
	// if reading from NULL, operation is aborted.
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
} tcp_cr_dest_t;

extern tcp_cr_dest_t   msg_cr_dest;

int tcp_parser(int sock);

#endif
