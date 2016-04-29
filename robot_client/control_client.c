/*
   Based on https://en.wikibooks.org/wiki/C_Programming/Networking_in_UNIX
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <pthread.h>
#include <signal.h>
#include <stdint.h>

#include <SDL/SDL.h>
 
#define IP_ADDR   INADDR_LOOPBACK
#define PORTNUM   8100

#define X_LEFT_AXIS     0
#define Y_LEFT_AXIS     1
#define X_RIGHT_AXIS 3
#define Y_RIGHT_AXIS 4

#define LT_BUMPER_AXIS  2
#define RT_BUMPER_AXIS  5

#define BTN_A  0
#define BTN_B  1
#define BTN_X  2
#define BTN_Y  3
#define LT_BTN 4
#define RT_BTN 5 // Doesn't work for some reason, maybe busted controller??
#define VIEW_BTN  6
#define MENU_BTN  7

SDL_Joystick *js;
int run_prog;

/*
   Attempts to open a joystick. If no joysticks are found, returns a negative number.
   Returns 0 otherwise to indicate success.
 */
int init_js(void)
{
   if (SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) < 0)
   {
      fprintf(stderr, "Couldn't initialize SDL: %s\n", SDL_GetError());
      exit(1);
   }

   SDL_JoystickEventState(SDL_ENABLE);
   js = SDL_JoystickOpen(0);
   if(!js)
   {
      fprintf(stderr, "Failed to open joystick!! Please check your connections \
                     and driver config and try again!!\n");
      return -1;
   }
   return 0;
}

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

void read_js(joystick_struct* s)
{
   SDL_Event event;
   SDL_PollEvent(&event);

   int btn, btn_st;

   switch(event.type)
   {
      case SDL_JOYAXISMOTION:
      switch(event.jaxis.axis)
      {
         case X_LEFT_AXIS:
            s->x_left = event.jaxis.value;
            break;
         case Y_LEFT_AXIS:
            s->y_left = event.jaxis.value;
            break;
         case X_RIGHT_AXIS:
            s->x_right = event.jaxis.value;
            break;
         case Y_RIGHT_AXIS:
            s->y_right = event.jaxis.value;
            break;
         case LT_BUMPER_AXIS:
            s->left_bumper = event.jaxis.value;
            break;
         case RT_BUMPER_AXIS:
            s->right_bumper = event.jaxis.value;
            break;
      }
      break;

      case SDL_JOYBUTTONDOWN:
         btn = event.jbutton.button;
         btn_st = event.jbutton.state;
         if(btn > -1 && btn < 11)
         {
            s->button_states |= (1<<(uint8_t)event.jbutton.button);
         }
         break;

      case SDL_JOYBUTTONUP:
         btn = event.jbutton.button;
         btn_st = event.jbutton.state;
         if(btn > -1 && btn < 11)
         {
            s->button_states &= ~(1<<(uint8_t)event.jbutton.button);
         }
         break;

      case SDL_JOYHATMOTION:
         break;

      default:
         break;
   }
}
 
int main(int argc, char *argv[])
{
   if(argc < 2)
   {
      printf("Usage: ./control_client [IP_ADDR]\n");
      return -1;
   }

   run_prog = 1;
   signal(SIGINT, sig_ctrlC_handler);
   init_js();
   joystick_struct joy0_data;

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

   union {
      joystick_struct s;
      uint8_t data_output[sizeof(joystick_struct)];
   } joystick_to_data;

   while(run_prog)
   {
      read_js(&(joystick_to_data.s));
      send(mysocket, "s", 1, 0);
      send(mysocket, joystick_to_data.data_output, sizeof(joystick_struct), 0);
      printf("%d\n", joystick_to_data.s.x_left);
      usleep((unsigned int)10000);
   }

   SDL_JoystickClose(js);
   close(mysocket);
   return EXIT_SUCCESS;
}