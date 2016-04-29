#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <signal.h>
 
#define PORTNUM 8100

int run_prog;

void sig_ctrlC_handler(int dummy_input)
{
    run_prog = 0;
}

/*
   Stores Joystick data in an organized format. Customized for XBox One controller
   with dual joysticks and analog bumpers:
 */
typedef struct {
   int16_t x_left;
   int16_t y_left;

   int16_t x_right;
   int16_t y_right;

   int16_t left_bumper;
   int16_t right_bumper;

   uint16_t button_states;
} joystick_struct;

typedef enum {
    ST_WAIT_FOR_CONN,
    ST_CONNECTED
} server_state;

int main(int argc, char *argv[])
{
    server_state srv_st = ST_WAIT_FOR_CONN;
    run_prog = 1;
    signal(SIGINT, sig_ctrlC_handler);
    /*
       Based on https://en.wikibooks.org/wiki/C_Programming/Networking_in_UNIX
     */
    char* msg = "Hello World!!\n";

    struct sockaddr_in dest; /* socket info about the machine connecting to us */
    struct sockaddr_in serv; /* socket info about our server */
    int mysocket;            /* socket used to listen for incoming connections */
    socklen_t socksize = sizeof(struct sockaddr_in);

    memset(&serv, 0, sizeof(serv));           /* zero the struct before filling the fields */
    serv.sin_family = AF_INET;                /* set the type of connection to TCP/IP */
    serv.sin_addr.s_addr = htonl(INADDR_ANY); /* set our address to any interface */
    serv.sin_port = htons(PORTNUM);           /* set the server port number */    

    mysocket = socket(AF_INET, SOCK_STREAM, 0);
  
    /* bind serv information to mysocket */
    bind(mysocket, (struct sockaddr *)&serv, sizeof(struct sockaddr));

    /* start listening, allowing a queue of up to 1 pending connection */

    /* Only 1 connection should be allowed anyway, since only 1 client 
       should have robot control anyway... 
     */
    listen(mysocket, 1);

    printf("Waiting for incoming connection\n");
    int consocket = accept(mysocket, (struct sockaddr *)&dest, &socksize);

    printf("Incoming connection from %s\n", inet_ntoa(dest.sin_addr));
    srv_st = ST_CONNECTED;

    union {
      joystick_struct joystick_data_output;
      uint8_t data_input[sizeof(joystick_struct)];
    } data_to_joystick;

    char st = 0;
    int len = 0;

    uint8_t incoming_data_buffer[sizeof(joystick_struct)];
    int recvd_bytes = 0;
    uint8_t incoming_data_chunk[sizeof(joystick_struct)];
    int chunk_len = 0;
    int i = 0;

    while(consocket && run_prog)
    {
        if(srv_st == ST_WAIT_FOR_CONN)
        {
            printf("Waiting for incoming connection\n"); 
            consocket = accept(mysocket, (struct sockaddr *)&dest, &socksize);
            srv_st = ST_CONNECTED;
        }

        len = recv(consocket, &st, 1, 0);
        if(len == 0)
        {
            /*A zero-byte recv() implies a dead TCP socket. Close it:*/
            close(consocket);
            printf("Connection dropped!! FD=%d\n", consocket);
            srv_st = ST_WAIT_FOR_CONN;
        }
        else
        {
            if(st == 's')
            {
                /*Try to receive sizeof(joystick_struct) bytes of data from our socket:*/
                while(recvd_bytes < sizeof(joystick_struct))
                {
                    chunk_len = recv(consocket, incoming_data_chunk, sizeof(joystick_struct), 0);

                    /*A zero-byte recv() implies a dead TCP socket. Close it:*/
                    if(chunk_len == 0)
                    {
                        close(consocket);
                        printf("Connection dropped!! FD=%d\n", consocket);
                        srv_st = ST_WAIT_FOR_CONN;
                        break;
                    }
                    else
                    {
                        for(i = 0; i < chunk_len; ++i)
                        {
                            data_to_joystick.data_input[recvd_bytes+i] = incoming_data_chunk[i];
                        }
                        recvd_bytes += chunk_len;
                    }
                }
                if(recvd_bytes == sizeof(joystick_struct))
                {
                    // printf("x_left: %d | y_left: %d | x_right: %d | y_right: %d\n", data_to_joystick.joystick_data_output.x_left,
                    //                                                                 data_to_joystick.joystick_data_output.y_left,
                    //                                                                 data_to_joystick.joystick_data_output.x_right,
                    //                                                                 data_to_joystick.joystick_data_output.y_right);

                    printf("Left bumper: %d | Right bumper: %d\n", data_to_joystick.joystick_data_output.left_bumper,
                                                                    data_to_joystick.joystick_data_output.right_bumper);
                    /* Act on incoming data and set commands to STM32 over UART: */
                }

                /*Reset counter:*/
                recvd_bytes = 0;
            }
            if(st == 'e')
            {
                printf("Shutting down control agent due to remote command!!\n");
                close(consocket);
                close(mysocket);
                return EXIT_SUCCESS;
            }
        }
    }

    close(mysocket);
    return EXIT_SUCCESS;
}