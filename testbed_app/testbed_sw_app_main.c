/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Modified by Fernando Cortes <fermando.corcam@gmail.com>
 * modified by Guillermo Rivera <memogrg@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
	C Standard Library/Newlib includes:
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
	HAL/HW-specific includes:
 */

#include <hal_common_includes.h>

/*
	Avionics software-specific includes:
 */

#include <interrupts.h>
#include <mission_timekeeper.h>
#include <imu.h>
#include <pwm_input.h>
#include <QuadRotor_PWM.h>
#include <comp_filter.h>
#include <pid_controller.h>
#include <lidar_lite_v1.h>

/*
	Shamelessly stolen from I2C example in libopencm3-examples,
	but sharing is caring, right? Right? Okay.
 */

// #define LRED GPIOE, GPIO9
// #define LORANGE GPIOE, GPIO10
// #define LGREEN GPIOE, GPIO11
#define LBLUE2 GPIOE, GPIO12
// #define LRED2 GPIOE, GPIO13
// #define LORANGE2 GPIOE, GPIO14
// #define LGREEN2 GPIOE, GPIO15

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA2,3 			-> 			USART2
	PE8,9,10,11,12,13,14,15 -> 	LEDs
	PB6,7 			-> 			I2C
	PA5,6,7 		-> 			SPI1 MISO,MOSI,CLK
	PE3 			-> 			SPI1 CS (user-controlled) for L3GD20
	PD3 			-> 			Timer2 Input capture
	PC6 			-> 			Timer3 Input capture
	PD12 			-> 			Timer4 Input capture
	PA15 			->			Timer8 Input capture
	PA8,9,10,PE14 	-> 			Timer1 PWM Output
 */

/*
 A basic routine to adjust system clock settings to get SYSCLK
 to 64 MHz, and have AHB buses at 64 MHz, and APB Bus at 32 MHz (its max speed)
 Copied from:
 https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f3/stm32f3-discovery/adc/adc.c
 */

/*
	External oscillator required to clock PLL at 72 MHz:
 */
static void set_system_clock(void)
{
	rcc_clock_setup_hsi(&hsi_8mhz[CLOCK_64MHZ]);
}

void setup_led_pins(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
	gpio_set(GPIOE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
}

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void init_user_button(void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
}

#ifdef INCLUDE_PWM_TEST_SHELL

static void pwm_test_shell(void)
{
	float pwm_setval = 0.04f;
	char cmd;
	uint8_t err = 0U;
	while(1)
	{
		scanf("%c", &cmd);
		switch(cmd)
		{
			case 'p':
			err=0U;
			pwm_setval += 0.001f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'n':
			err=0U;
			pwm_setval -= 0.001f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'o':
			err=0U;
			pwm_setval += 0.10f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'b':
			err=0U;
			pwm_setval -= 0.10f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			case 'a':
			err=0U;
			QuadRotor_motor1_start();
			QuadRotor_motor2_start();
			QuadRotor_motor3_start();
			QuadRotor_motor4_start();
			break;
			case 's':
			err=0U;
			QuadRotor_motor1_stop();
			QuadRotor_motor2_stop();
			QuadRotor_motor3_stop();
			QuadRotor_motor4_stop();
			break;
			case 'z':
			err=0U;
			pwm_setval = 0.010f;
			QuadRotor_motor1_setDuty(pwm_setval);
			QuadRotor_motor2_setDuty(pwm_setval);
			QuadRotor_motor3_setDuty(pwm_setval);
			QuadRotor_motor4_setDuty(pwm_setval);
			break;
			default:
			err=1;
			break;
		}
		if(!err)
		{
			printf("%c %f\r\n", cmd, pwm_setval);
		}
		else
		{
			printf("Usage: [n,p]: Decrease/Increase PWM fine precision\r\n");
			printf("[b,o]: Decrease/Increase PWM course precision\r\n");
			printf("[a] to start output, [s] to [s]top output\r\n");
			printf("[z] to set approx 1.07 ms pulse output and keep output enabled\r\n");
		}
	}
}

#endif

/*
	Returns 0 if successful
	Returns -1 if invalid angle set
 */

	/* 

	Joint 5:
	 0.1 = 0 Degrees in my coord system
	 0.053 = 90 Degrees in my coord system

	Joint 6:
	 0 Deg = 0.06
	 90 Deg = 0.107

	Joint 7:
	 0 Deg: 0.054
	 90 Deg: 0.103

	Joint 8:
	 0 Deg: 0.097
	 90 Deg: 0.050

	*/

float map90(float zero_val, float perp_val, float angle_desired)
{
	return (angle_desired*(perp_val-zero_val)/(float)90.0f) + zero_val;
}

float map180(float zero_val, float perp_val, float angle_desired)
{
	return (angle_desired*(perp_val-zero_val)/(float)180.0f) + zero_val;
}

int set_angle(int joint, float angle)
{
	if(angle < 0.0f)
	{
		return -1;
	}
	if(joint <= 4 && angle > 185.0f)
	{
		return -1;
	}
	if(joint > 4 && angle > 95.0f)
	{
		return -1;
	}

	// Else:
	switch(joint)
	{
		case 1:
			QuadRotor_motor1_setDuty(map180(0.122f, 0.036f, angle));
			break;
		case 2:
			QuadRotor_motor2_setDuty(map180(0.13f, 0.05f, angle));
			break;
		case 3:
			QuadRotor_motor3_setDuty(map180(0.12f, 0.034f, angle));
			break;
		case 4:
			QuadRotor_motor4_setDuty(map180(0.12f, 0.034f, angle));
			break;
		case 5:
			QuadRotor_motor5_setDuty(map90(0.085f, 0.042f, angle));
			break;
		case 6:
			QuadRotor_motor6_setDuty(map90(0.057f, 0.101f, angle));
			break;
		case 7:
			QuadRotor_motor8_setDuty(map90(0.090f, 0.048f, angle));
			break;
		case 8:
			QuadRotor_motor7_setDuty(map90(0.05f, 0.096f, angle));
			break;
		default:
			return -1;
	}
	return 0;
}

#define BASIC_GAIT			1
// #define EXPERIMENTAL_GAIT_1	1

void simple_fwd_gait(void)
{
	float rt = -25.0f;
	float m1_fwd_max = 175;
	float m1_fwd_min = 57+rt;

	float m4_fwd_max = 98+rt;//95;
	float m4_fwd_min = 0;

	float m2_fwd_max = 80;
	float m2_fwd_min = 5;

	float m3_fwd_max = 145;
	float m3_fwd_min = 70;

	// set_angle(1, 180);
	// set_angle(2, 90);
	// set_angle(3, 90);
	// set_angle(4, 90);
	// set_angle(5, 90);
	// set_angle(6, 90);
	// set_angle(7, 90);
	// set_angle(8, 90);
	// while(1);

	// float m1_fwd_max = 175;
	// float m1_fwd_min = 60;

	// float m4_fwd_max = 95;
	// float m4_fwd_min = 0;

	// float m2_fwd_max = 60;//80;
	// float m2_fwd_min = 15;//5;

	// float m3_fwd_max = 145;//120;
	// float m3_fwd_min = 70;//95;

	uint16_t speed_inv = 1U;

	#ifdef BASIC_GAIT
		set_angle(6, 20.0f);
		timekeeper_delay(100U);
		set_angle(2, m2_fwd_max);
		timekeeper_delay(100U);
		set_angle(1, m1_fwd_max);
		timekeeper_delay(15U);
		set_angle(6, 50.0f);
		timekeeper_delay(100U);
		set_angle(7, 20.0f);
		timekeeper_delay(150U);
		set_angle(3, m3_fwd_max);
		timekeeper_delay(100U);
		set_angle(4, m4_fwd_max);
		timekeeper_delay(100U);
		set_angle(7, 50.0f);
		timekeeper_delay(100U);
		
		set_angle(5, 20.0f);
		timekeeper_delay(100U);
		set_angle(1, m1_fwd_min);
		timekeeper_delay(250U); //
		set_angle(2, m2_fwd_min);
		timekeeper_delay(15U);
		set_angle(5, 50.0f);
		timekeeper_delay(15U);
		set_angle(8, 20.0f);
		timekeeper_delay(250U); //
		set_angle(4, m4_fwd_min);
		timekeeper_delay(100U);
		set_angle(3, m3_fwd_min);
		timekeeper_delay(15U);
		set_angle(8, 50.0f);

		// set_angle(6, 10.0f);
		// timekeeper_delay(150U);
		// set_angle(2, m2_fwd_max);
		// timekeeper_delay(150U);
		// set_angle(1, m1_fwd_max);
		// set_angle(6, 50.0f);

		// set_angle(7, 5.0f);
		// timekeeper_delay(150U);
		// set_angle(3, m3_fwd_max);
		// timekeeper_delay(150U);
		// set_angle(4, m4_fwd_max);
		// set_angle(7, 50.0f);
		// timekeeper_delay(150U);
		
		// set_angle(5, 5.0f);
		// timekeeper_delay(175U);
		// set_angle(1, m1_fwd_min);
		// timekeeper_delay(150U);
		// set_angle(2, m2_fwd_min);
		// set_angle(5, 50.0f);

		// set_angle(8, 5.0f);
		// timekeeper_delay(150U);
		// set_angle(4, m4_fwd_min);
		// timekeeper_delay(150U);
		// set_angle(3, m3_fwd_min);
		// set_angle(8, 50.0f);
	#endif

	#ifdef EXPERIMENTAL_GAIT_1
		/*Move front left leg up, forward and back down*/
		set_angle(5, 5.0f);
		timekeeper_delay(150U*speed_inv);
		set_angle(1, 100.0f);
		timekeeper_delay(150U*speed_inv);
		set_angle(5, 70.0f);

		/*Wait a short time*/
		timekeeper_delay(50U*speed_inv);

		/*Move rear left leg up, forward and back down*/
		set_angle(8, 5.0f);
		timekeeper_delay(150U*speed_inv);
		set_angle(4, 10.0f);
		timekeeper_delay(200U*speed_inv);
		set_angle(8, 50.0f);

		/*Wait a short time*/
		timekeeper_delay(50U*speed_inv);

		/*Move front and rear legs both backward to "push" that side forward*/
		set_angle(1, 170.0f);
		set_angle(4, 50.0f);
		timekeeper_delay(150U*speed_inv);

		/*Move rear right leg up, forward and back down*/
		set_angle(7, 5.0f);
		timekeeper_delay(150U*speed_inv);
		set_angle(3, 160.0f);
		timekeeper_delay(150U*speed_inv);
		set_angle(7, 50.0f);

		/*Wait a short time*/
		timekeeper_delay(50U*speed_inv);

		/*Move front right leg up, forward and back down*/
		set_angle(6, 5.0f);
		timekeeper_delay(150U*speed_inv);
		set_angle(2, 100.0f);
		timekeeper_delay(180U*speed_inv);
		set_angle(6, 50.0f);

		/*Wait a short time*/
		timekeeper_delay(50U*speed_inv);

		/*Move front and rear legs both backward to "push" that side forward*/
		set_angle(2, 10.0f);
		set_angle(3, 80.0f);
		timekeeper_delay(150U*speed_inv);
	#endif
}

#define MOTOR1 	0
#define MOTOR2 	1
#define MOTOR3 	2
#define MOTOR4 	3
#define MOTOR5 	4
#define MOTOR6 	5
#define MOTOR7 	6
#define MOTOR8 	7

int main(void)
{
	_disable_interrupts();

		imu_scaled_data_struct imu_struct_scaled;
		imu_raw_data_struct imu_struct_raw;

		rc_joystick_data_struct js;

		set_system_clock();
		setup_led_pins();
		systick_setup();
		usart_setup();
		init_user_button();

		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

		init_mission_timekeeper();
		initialize_imu(SCALE_2G, SCALE_1POINT9_GAUSS, SCALE_250_DPS, &imu_struct_scaled);
		
	_enable_interrupts();

	timekeeper_delay(500U);

	#ifdef ENABLE_PWM_TEST_SHELL

			/*
				Simple shell for testing PWM functionality:
			 */
			_disable_interrupts();

			QuadRotor_PWM_init();
			QuadRotor_motor1_start();
			QuadRotor_motor2_start();
			QuadRotor_motor3_start();
			QuadRotor_motor4_start();
			
			_enable_interrupts();

			pwm_test_shell();

	#endif

	QuadRotor_PWM_init();

	QuadRotor_motor1_start();
	QuadRotor_motor2_start();
	QuadRotor_motor3_start();
	QuadRotor_motor4_start();

	QuadRotor_motor5_start();
	QuadRotor_motor6_start();
	QuadRotor_motor7_start();
	QuadRotor_motor8_start();

	printf("Ready\r\n");

	QuadRotor_motor1_setDuty(0.5f);
	QuadRotor_motor2_setDuty(0.5f);
	QuadRotor_motor3_setDuty(0.5f);
	QuadRotor_motor4_setDuty(0.5f);

	// uint8_t user_output_flag = create_flag(10U);

	float corrected_gyro_data[3];

	// float angle = 0.0f;

	float motor1_angle = 90.0f;
	float motor2_angle = 90.0f;
	float motor3_angle = 90.0f;
	float motor4_angle = 90.0f;

	float motor5_angle = 45.0f;
	float motor6_angle = 45.0f;
	float motor7_angle = 45.0f;
	float motor8_angle = 45.0f;

	float motor1_angle_offset = 0.0f;
	float motor2_angle_offset = 0.0f;
	float motor3_angle_offset = 0.0f;
	float motor4_angle_offset = 0.0f;
	float motor5_angle_offset = 0.8f;
	float motor6_angle_offset = 0.4f;
	float motor7_angle_offset = 0.8f;
	float motor8_angle_offset = 0.4f;

	set_angle(1, 135.0f);
	set_angle(2, 45.0f);
	set_angle(3, 135.0f);
	set_angle(4, 45.0f);

	set_angle(5, 45.0f);
	set_angle(6, 45.0f);
	set_angle(7, 45.0f);
	set_angle(8, 45.0f);

	float angle = 0.0f;

	// Wait for user start:
	while(!gpio_get(GPIOA, GPIO0));

	while (1)
	{
		simple_fwd_gait();

		// if(get_flag_state(user_output_flag) == STATE_PENDING)
		// {
		// 	reset_flag(user_output_flag);

			/* For a simple push-up and come back down cyclic demo: */

			// angle = push_up_advance_state();

			// set_angle(5, angle);
			// set_angle(6, angle);
			// set_angle(7, angle);
			// set_angle(8, angle);

			// motor1_angle = 135.0f + (30.0f*sin(angle+motor1_angle_offset));
			// motor5_angle = 45.0f + (20.0f*sin(1.8f*(angle+motor5_angle_offset)));

			// motor2_angle = 45.0f + (30.0f*sin(angle+motor2_angle_offset));
			// motor6_angle = 45.0f + (20.0f*sin((1.8f*angle+motor6_angle_offset)));
			
			// motor3_angle = 135.0f - (30.0f*sin(angle+motor3_angle_offset));
			// motor7_angle = 45.0f + (20.0f*sin((1.8f*angle+motor7_angle_offset)));
			
			// motor4_angle = 45.0f - (30.0f*sin(angle+motor4_angle_offset));
			// motor8_angle = 45.0f + (20.0f*sin((1.8f*angle+motor8_angle_offset)));

			// angle += 0.01f;
			// if(angle > 2.0f*3.14159f)
			// {
			// 	angle = 0.0f;
			// }

			// set_angle(1, motor1_angle);
			// set_angle(5, motor5_angle);

			// set_angle(2, motor2_angle);
			// set_angle(6, motor6_angle);
			
			// set_angle(3, motor3_angle);
			// set_angle(7, motor7_angle);
			
			// set_angle(4, motor4_angle);
			// set_angle(8, motor8_angle);



		// }
	}

	return 0;
}

int push_up_advance_state(void)
{
	static float angle = 0.0f;
	static int up = 1;

	if(angle > 90.0f)
	{
		up = 0;
	}
	if(angle < 0.0f)
	{
		up = 1;
	}

	if(up == 0)
	{
		angle -= 0.5f;
	}
	if(up == 1)
	{
		angle += 0.5f;
	}

	return angle;
}

void simple_walker(int* lookup_table, int* angle_commands)
{

}
