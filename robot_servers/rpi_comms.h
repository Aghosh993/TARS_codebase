#ifndef RPI_COMMS_H
#define RPI_COMMS_H	1

#include <stdlib.h>
#include <stdint.h>

#define SERVO_PWR_EN_MASK	(1<<0)
#define M1_HIGHSIDE_MASK	(1<<1)
#define M2_HIGHSIDE_MASK	(1<<2)

typedef struct {
	int8_t fwd_speed;
	int8_t turn_amt;
	uint8_t movement_mode;
	uint8_t turret_pan;
	uint8_t turret_tilt;
	uint8_t logic_control_states;
} incoming_command_packet;

typedef struct {
	uint16_t batt_voltage;
	uint16_t servo_bus_voltage;
	uint8_t alerts;
} outgoing_status_packet;

typedef enum {
	STATE_IDLE,
	STATE_GETTING_DATA
} incoming_reception_state;

void init_pi_comms(void);
void process_incoming_data(uint8_t c);
incoming_command_packet get_last_cmd_packet(void);
void create_command_transmission(int8_t speed, int8_t turn, uint8_t mode, uint8_t pan, uint8_t tilt, uint8_t control_states, uint8_t *data_buffer);
void create_status_transmission(uint16_t battery_voltage, uint16_t servo_voltage, uint8_t alerts, uint8_t *outgoing_data_buffer);

#endif