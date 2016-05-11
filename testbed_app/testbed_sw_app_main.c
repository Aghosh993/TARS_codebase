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
#include <robot_config.h>

/*
	Avionics software-specific includes:
 */

#include <interrupts.h>
#include <mission_timekeeper.h>
#include <imu.h>
#include <QuadRotor_PWM.h>
#include <rpi_comms.h>

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

#define BTN_A  0
#define BTN_B  1
#define BTN_X  2
#define BTN_Y  3
#define LT_BTN 4
#define RT_BTN 5 // Doesn't work for some reason, maybe busted controller??
#define VIEW_BTN  6
#define MENU_BTN  7
#define LEFT_STICK_BUTTON   8
#define RIGHT_STICK_BUTTON  9

// #define INCLUDE_PWM_TEST_SHELL	1
// #define ENABLE_PWM_TEST_SHELL	1

/*
	Summary of hardware pin usage:

	PA9,10 			->			USART1, DEBUG
	PA2,3 			-> 			USART2, Raspberry Pi
	PE9,11,13,14 	-> 			Timer1 PWM Output
	PC6,7,PD6,7 	-> 			Timer3 PWM Output
	PD2 			-> 			Servo Power enable
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
	// rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_64MHZ]);
}

void setup_led_pins(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
	gpio_set(GPIOE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
}

void usart1_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_USART1);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO10);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9 | GPIO10);

	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	// Unmask receive interrupt
	usart_enable_rx_interrupt(USART1);
	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_USART1_EXTI25_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART1);
}

void usart2_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO2 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	
	// Unmask receive interrupt
	usart_enable_rx_interrupt(USART2);
	// Make sure the interrupt is routed through the NVIC
	nvic_enable_irq(NVIC_USART2_EXTI26_IRQ);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void usart_setup(void)
{
	usart1_setup();
	usart2_setup();
}

static void init_user_button(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, GPIO15);
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

/* 
	rt is a floating-point argument from -25.0f to 25.0f that allows the vehicle to
    steer while it walks forward:
 */
void simple_fwd_gait(float rt)
{
	float m1_fwd_max = 175;
	float m1_fwd_min = 87+rt;

	float m4_fwd_max = 98+rt;//95;
	float m4_fwd_min = 0;

	float m2_fwd_max = 80;
	float m2_fwd_min = 5;

	float m3_fwd_max = 145;
	float m3_fwd_min = 70;

	uint16_t speed_inv = 1U;

	#ifdef BASIC_GAIT
		// set_angle(6, 20.0f);
		// timekeeper_delay(100U);
		// set_angle(2, m2_fwd_max);
		// timekeeper_delay(100U);
		// set_angle(1, m1_fwd_max);
		// timekeeper_delay(15U); //??
		// set_angle(6, 50.0f);
		// timekeeper_delay(100U);
		// set_angle(7, 20.0f);
		// timekeeper_delay(150U);
		// set_angle(3, m3_fwd_max);
		// timekeeper_delay(100U);
		// set_angle(4, m4_fwd_max);
		// timekeeper_delay(100U);
		// set_angle(7, 50.0f);
		// timekeeper_delay(100U);
		
		// set_angle(5, 20.0f);
		// timekeeper_delay(200U);
		// set_angle(1, m1_fwd_min);
		// timekeeper_delay(250U); //
		// set_angle(2, m2_fwd_min);
		// timekeeper_delay(15U);
		// set_angle(5, 50.0f);
		// timekeeper_delay(15U);
		// set_angle(8, 10.0f);
		// timekeeper_delay(200U); //
		// set_angle(4, m4_fwd_min);
		// timekeeper_delay(100U);
		// set_angle(3, m3_fwd_min);
		// timekeeper_delay(15U);
		// set_angle(8, 50.0f);

		// Based on latest Git code:

		// set_angle(6, 20.0f);
		// timekeeper_delay(100U);
		// set_angle(2, m2_fwd_max);
		// timekeeper_delay(100U);
		// set_angle(1, m1_fwd_max);
		// timekeeper_delay(15U);
		// set_angle(6, 50.0f);
		// timekeeper_delay(100U);
		// set_angle(7, 20.0f);
		// timekeeper_delay(150U);
		// set_angle(3, m3_fwd_max);
		// timekeeper_delay(100U);
		// set_angle(4, m4_fwd_max);
		// timekeeper_delay(100U);
		// set_angle(7, 50.0f);
		// timekeeper_delay(100U);
		
		// set_angle(5, 20.0f);
		// timekeeper_delay(250U);
		// set_angle(1, m1_fwd_min);
		// timekeeper_delay(250U); //
		// set_angle(2, m2_fwd_min);
		// timekeeper_delay(15U);
		// set_angle(5, 50.0f);
		// timekeeper_delay(15U);
		// set_angle(8, 20.0f);
		// timekeeper_delay(200U); //
		// set_angle(4, m4_fwd_min);
		// timekeeper_delay(100U);
		// set_angle(3, m3_fwd_min);
		// timekeeper_delay(15U);
		// set_angle(8, 50.0f);

		// Based on Git first stable commit:
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
	#endif

	// #ifdef EXPERIMENTAL_GAIT_1
	// 	/*Move front left leg up, forward and back down*/
	// 	set_angle(5, 5.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// 	set_angle(1, 100.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// 	set_angle(5, 70.0f);

	// 	/*Wait a short time*/
	// 	timekeeper_delay(50U*speed_inv);

	// 	/*Move rear left leg up, forward and back down*/
	// 	set_angle(8, 5.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// 	set_angle(4, 10.0f);
	// 	timekeeper_delay(200U*speed_inv);
	// 	set_angle(8, 50.0f);

	// 	/*Wait a short time*/
	// 	timekeeper_delay(50U*speed_inv);

	// 	/*Move front and rear legs both backward to "push" that side forward*/
	// 	set_angle(1, 170.0f);
	// 	set_angle(4, 50.0f);
	// 	timekeeper_delay(150U*speed_inv);

	// 	/*Move rear right leg up, forward and back down*/
	// 	set_angle(7, 5.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// 	set_angle(3, 160.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// 	set_angle(7, 50.0f);

	// 	/*Wait a short time*/
	// 	timekeeper_delay(50U*speed_inv);

	// 	/*Move front right leg up, forward and back down*/
	// 	set_angle(6, 5.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// 	set_angle(2, 100.0f);
	// 	timekeeper_delay(180U*speed_inv);
	// 	set_angle(6, 50.0f);

	// 	/*Wait a short time*/
	// 	timekeeper_delay(50U*speed_inv);

	// 	/*Move front and rear legs both backward to "push" that side forward*/
	// 	set_angle(2, 10.0f);
	// 	set_angle(3, 80.0f);
	// 	timekeeper_delay(150U*speed_inv);
	// #endif
}

#ifdef HIGHSIDE_SWITCHES_USE_PWM
	static void set_m1_highside_switch(float value)
	{
		uint32_t duty = (uint32_t)((float)64000 * value);
		timer_set_oc_value(TIM2, TIM_OC3, duty);
	}

	static void set_m2_highside_switch(float value)
	{
		uint32_t duty = (uint32_t)((float)64000 * value);
		timer_set_oc_value(TIM2, TIM_OC4, duty);
	}
#endif

#ifndef HIGHSIDE_SWITCHES_USE_PWM
	static void setup_highside_switches(void)
	{
		// Set up high-side switch control channels as GPIOs
		gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6 | GPIO7);
		// Initialize the outputs to logic LOW to inhibit both channels:
		gpio_clear(GPIOD, GPIO6 | GPIO7);
	}
	static void set_m1_highside_switch(int value)
	{
		if(value>0)
		{
			gpio_set(GPIOD, GPIO6);
		}
		else
		{
			gpio_clear(GPIOD, GPIO6);
		}
	}

	static void set_m2_highside_switch(int value)
	{
		if(value>0)
		{
			gpio_set(GPIOD, GPIO7);
		}
		else
		{
			gpio_clear(GPIOD, GPIO7);
		}
	}
#endif

static void setup_servo_enable_outut(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO2);
	gpio_clear(GPIOD, GPIO2);
}

static void enable_servo_bus(void)
{
	gpio_set(GPIOD, GPIO2);
}

static void disable_servo_bus(void)
{
	gpio_clear(GPIOD, GPIO2);
}

static void rpi_send_buffer(int len, uint8_t *buffer)
{
	int i = 0;
	for(i=0; i<len; ++i)
	{
		usart_send_blocking(USART2, buffer[i]);
	}
}

static void init_adc_input(void)
{
	//ADC
	rcc_periph_clock_enable(RCC_ADC34);
	rcc_periph_clock_enable(RCC_GPIOB);
	//ADC
	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
	adc_off(ADC3);
	RCC_CFGR2 |= (uint32_t)0x10003400;
	adc_set_clk_prescale(ADC_CCR_CKMODE_DIV4);
	adc_set_single_conversion_mode(ADC3);
	adc_disable_external_trigger_regular(ADC3);
	adc_set_right_aligned(ADC3);

	adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR1_SMP_1DOT5CYC);
	adc_set_resolution(ADC3, ADC_CFGR_RES_12_BIT);
	adc_power_on(ADC3);

	/* Wait for ADC starting up. */
	int i;
	for (i = 0; i < 800000; i++)
		__asm__("nop");

	uint8_t channel_array[16];
	int j = 0;
	for(j=0; j<16; ++j)
	{
		channel_array[j]=1;	
	}
	
	adc_set_regular_sequence(ADC3, 1, channel_array);
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

		set_system_clock();
		setup_led_pins();
		systick_setup();
		usart_setup();
		init_user_button();

		setvbuf(stdin,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)
		setvbuf(stdout,NULL,_IONBF,0); 	// Sets stdin in unbuffered mode (normal for usart com)

		init_mission_timekeeper();
		init_pi_comms();
		
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

	QuadRotor_motor1_setDuty(0.5f);
	QuadRotor_motor2_setDuty(0.5f);
	QuadRotor_motor3_setDuty(0.5f);
	QuadRotor_motor4_setDuty(0.5f);

	/*
		Enable highside switches as logic channels:
	 */
	#ifndef HIGHSIDE_SWITCHES_USE_PWM
		setup_highside_switches();
	#endif

	/*
		Enable 5 Volt servo power bus by setting logic output to its highside switch:
	 */
	setup_servo_enable_outut();
	enable_servo_bus();

	timekeeper_delay(1000U);

	set_angle(1, 135.0f);
	set_angle(2, 45.0f);
	set_angle(3, 135.0f);
	set_angle(4, 45.0f);

	set_angle(5, 45.0f);
	set_angle(6, 45.0f);
	set_angle(7, 45.0f);
	set_angle(8, 45.0f);

	float angle = 0.0f;

	uint16_t tilt_val = 2700U;
	uint16_t pan_val = 58880U;

	timer_set_oc_value(TIM2, TIM_OC1, tilt_val);	//Tilt
	timer_set_oc_value(TIM2, TIM_OC2, pan_val);		//Pan
	timekeeper_delay(1000U);
	timer_disable_oc_output(TIM2, TIM_OC1);
	timer_disable_oc_output(TIM2, TIM_OC2);

	/*
		Initialize voltage monitor ADC channels:
	 */
	// init_adc_input();

	printf("Starting...\r\n");

	float turn_amt = 8.0f; // +ve turns right, -ve turns left...
	int adcval = 0;
	incoming_command_packet cmd;

	timekeeper_delay(5000U);

	while (1)
	{
		cmd = get_last_cmd_packet();
		turn_amt = (float)cmd.turn_amt/(float)2.3f;
		if(cmd.movement_mode == 1U)
		{
			simple_fwd_gait(turn_amt);
		}

		if((cmd.logic_control_states & (1<<LT_BTN)))// || (cmd.logic_control_states & (1<<RT_BTN)))
		{
			#ifdef HIGHSIDE_SWITCHES_USE_PWM
				set_m1_highside_switch(0.80f);
				set_m2_highside_switch(0.80f);
			#endif

			#ifndef HIGHSIDE_SWITCHES_USE_PWM
				set_m1_highside_switch(1);
				set_m2_highside_switch(1);
			#endif
		}
		else
		{
			#ifdef HIGHSIDE_SWITCHES_USE_PWM
				set_m1_highside_switch(0.00f);
				set_m2_highside_switch(0.00f);
			#endif

			#ifndef HIGHSIDE_SWITCHES_USE_PWM
				set_m1_highside_switch(0);
				set_m2_highside_switch(0);
			#endif			
		}

		if(cmd.logic_control_states & (1<<BTN_A))
		{
			tilt_val -= 10U;
			timer_enable_oc_output(TIM2, TIM_OC1);
			timer_set_oc_value(TIM2, TIM_OC1, tilt_val);	//Tilt
			timekeeper_delay(100U);
			timer_disable_oc_output(TIM2, TIM_OC1);
		}

		if(cmd.logic_control_states & (1<<BTN_Y))
		{
			tilt_val += 10U;
			timer_enable_oc_output(TIM2, TIM_OC1);
			timer_set_oc_value(TIM2, TIM_OC1, tilt_val);	//Tilt
			timekeeper_delay(100U);
			timer_disable_oc_output(TIM2, TIM_OC1);	
		}

		if(cmd.logic_control_states & (1<<BTN_X))
		{
			pan_val -= 10U;
			timer_enable_oc_output(TIM2, TIM_OC2);
			timer_set_oc_value(TIM2, TIM_OC2, pan_val);	//Pan
			timekeeper_delay(100U);
			timer_disable_oc_output(TIM2, TIM_OC2);
		}

		if(cmd.logic_control_states & (1<<BTN_B))
		{
			pan_val += 10U;
			timer_enable_oc_output(TIM2, TIM_OC2);
			timer_set_oc_value(TIM2, TIM_OC2, pan_val);	//Pan
			timekeeper_delay(100U);
			timer_disable_oc_output(TIM2, TIM_OC2);	
		}

		// Simple test of high-side switches for the DC brushed motor outputs:
		// gpio_set(GPIOD, GPIO6 | GPIO7);
		// timekeeper_delay(500U);
		// gpio_clear(GPIOD, GPIO6 | GPIO7);
		// timekeeper_delay(500U);

		// Testing airsoft control channels:
		// if(gpio_get(GPIOD, GPIO15) > 0)
		// {
		// 	#ifdef HIGHSIDE_SWITCHES_USE_PWM
		// 		set_m1_highside_switch(0.80f);
		// 		set_m2_highside_switch(0.80f);
		// 	#endif

		// 	#ifndef HIGHSIDE_SWITCHES_USE_PWM
		// 		set_m1_highside_switch(1);
		// 		set_m2_highside_switch(1);
		// 	#endif
		// }
		// else
		// {
		// 	#ifdef HIGHSIDE_SWITCHES_USE_PWM
		// 		set_m1_highside_switch(0.00f);
		// 		set_m2_highside_switch(0.00f);
		// 	#endif

		// 	#ifndef HIGHSIDE_SWITCHES_USE_PWM
		// 		set_m1_highside_switch(0);
		// 		set_m2_highside_switch(0);
		// 	#endif
		// }
		// printf("%d\r\n", gpio_get(GPIOD, GPIO15));
		printf("%d %d %d %d %d %d %s\r\n", cmd.fwd_speed, cmd.turn_amt, cmd.movement_mode, 
											cmd.turret_pan, cmd.turret_tilt, cmd.logic_control_states,
											cmd.rpi_ip_addr_string);
		// while(!adc_eoc(ADC2));
		// adcval = adc_read_regular(ADC2);
		// printf("%f\r\n", (float)7.045f*(float)adcval*(float)3.0f/(float)4096.0f);
		// adc_start_conversion_regular(ADC2);
	}

	return 0;
}
