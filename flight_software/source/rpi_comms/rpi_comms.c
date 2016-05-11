#include <rpi_comms.h>

static incoming_reception_state st;
static int recv_buffer_iterator;
static uint8_t recv_buffer[sizeof(incoming_command_packet)];
static incoming_command_packet p;

static void parse_incoming_buffer(uint8_t *input, incoming_command_packet *pack)
{
	union {
		uint8_t in_data[sizeof(incoming_command_packet)];
		incoming_command_packet out_data;
	} data_to_packet;

	int i = 0;
	for(i=0; i<sizeof(incoming_command_packet); ++i)
	{
		data_to_packet.in_data[i] = input[i];
	}

	pack->fwd_speed = data_to_packet.out_data.fwd_speed;
	pack->turn_amt = data_to_packet.out_data.turn_amt;
	pack->movement_mode = data_to_packet.out_data.movement_mode;
	pack->turret_pan = data_to_packet.out_data.turret_pan;
	pack->turret_tilt = data_to_packet.out_data.turret_tilt;
	pack->logic_control_states = data_to_packet.out_data.logic_control_states;

	strncpy(pack->rpi_ip_addr_string, data_to_packet.out_data.rpi_ip_addr_string, 17U);
}

void init_pi_comms(void)
{
	st = STATE_IDLE;
	recv_buffer_iterator = 0;
}

void process_incoming_data(uint8_t c)
{
	switch(st)
	{
		case STATE_IDLE:
			if(c=='s')
			{
				st = STATE_GETTING_DATA;
			}
			break;
		case STATE_GETTING_DATA:
			if(recv_buffer_iterator < sizeof(incoming_command_packet))
			{
				recv_buffer[recv_buffer_iterator] = c;
				++recv_buffer_iterator;
			}
			else
			{
				parse_incoming_buffer(recv_buffer, &p);
				recv_buffer_iterator = 0;
				st = STATE_IDLE;
			}
			break;
	}
}

incoming_command_packet get_last_cmd_packet(void)
{
	return p;
}

void create_command_transmission(int8_t speed, int8_t turn, uint8_t mode, uint8_t pan, uint8_t tilt, uint8_t control_states, uint8_t* ip_addr, uint8_t *data_buffer)
{
	union {
		incoming_command_packet in_data;
		uint8_t out_data[sizeof(incoming_command_packet)];
	} cmd_packet_to_data;

	cmd_packet_to_data.in_data.fwd_speed = speed;
	cmd_packet_to_data.in_data.turn_amt = turn;
	cmd_packet_to_data.in_data.movement_mode = mode;
	cmd_packet_to_data.in_data.turret_pan = pan;
	cmd_packet_to_data.in_data.turret_tilt = tilt;
	cmd_packet_to_data.in_data.logic_control_states = control_states;

	strncpy(cmd_packet_to_data.in_data.rpi_ip_addr_string, ip_addr, 17U);

	int i = 0;

	data_buffer[0] = 's';

	for(i=0; i<sizeof(incoming_command_packet); ++i)
	{
		data_buffer[i+1U] = cmd_packet_to_data.out_data[i];
	}
}

void create_status_transmission(uint16_t battery_voltage, uint16_t servo_voltage, uint8_t alerts_byte, uint8_t *outgoing_data_buffer)
{
	union {
		outgoing_status_packet in_data;
		uint8_t out_data[sizeof(outgoing_status_packet)];
	} status_packet_to_data;

	status_packet_to_data.in_data.batt_voltage = battery_voltage;
	status_packet_to_data.in_data.servo_bus_voltage = servo_voltage;
	status_packet_to_data.in_data.alerts = alerts_byte;
	
	int i = 0;

	outgoing_data_buffer[0] = 's';

	for(i=0; i<sizeof(outgoing_status_packet); ++i)
	{
		outgoing_data_buffer[i+1U] = status_packet_to_data.out_data[i];
	}
}
