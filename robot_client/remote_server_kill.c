/*
   Based on https://en.wikibooks.org/wiki/C_Programming/Networking_in_UNIX
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#define PORTNUM   8100

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		printf("Usage: ./servkill [IP_ADDR]\n");
		return -1;
	}
	int len, mysocket;
	struct sockaddr_in dest;

	mysocket = socket(AF_INET, SOCK_STREAM, 0);

	memset(&dest, 0, sizeof(dest));                /* zero the struct */
	dest.sin_family = AF_INET;
	dest.sin_addr.s_addr = inet_addr(argv[1]); 
	dest.sin_port = htons(PORTNUM);                /* set destination port number */

	int conn_res = connect(mysocket, (struct sockaddr *)&dest, sizeof(struct sockaddr));

	switch(errno)
	{
	  case ECONNREFUSED:
	     printf("Connection refused! Aborting!!\n");
	     return -1;
	  case EADDRNOTAVAIL:
	     printf("Address not available from this machine, aborting!!\n");         
	  case EALREADY:
	     printf("Connection already in progress on socket, aborting!!\n");
	  case ENETUNREACH:
	     printf("No route to address present on network, aborting!!\n");
	  default:
	     if(conn_res < 0)
	     {
	        printf("Unknown error, aborting!!\n");
	        return -1;
	     }
	}

	send(mysocket, "e", 1, 0);

	close(mysocket);

	return EXIT_SUCCESS;
}