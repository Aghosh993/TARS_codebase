#include "QuadRotor_PWM_hal.h"

static void setup_timer1_pwm(void)
{
	rcc_periph_clock_enable(RCC_GPIOE);
	gpio_set_output_options(GPIOE, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO9 | GPIO11 | GPIO13 | GPIO14);
	gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO13 | GPIO14);

	gpio_set_af(GPIOE, GPIO_AF2, GPIO9 | GPIO11 | GPIO13 | GPIO14);

	timer_reset(TIM1);
	rcc_periph_clock_enable(RCC_TIM1);

	timer_continuous_mode(TIM1);

	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
	           TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM1, TIM_OC4, TIM_OCM_PWM1);

	timer_set_period(TIM1, 64000);
	timer_set_prescaler(TIM1, 19); // for 50 Hz, this needs to be 1 for 500 Hz output

	timer_set_oc_value(TIM1, TIM_OC1, 0);
	timer_set_oc_value(TIM1, TIM_OC2, 0);
	timer_set_oc_value(TIM1, TIM_OC3, 0);
	timer_set_oc_value(TIM1, TIM_OC4, 0);

	timer_enable_preload(TIM1);

	timer_set_oc_polarity_high(TIM1, TIM_OC1);
	timer_set_oc_polarity_high(TIM1, TIM_OC2);
	timer_set_oc_polarity_high(TIM1, TIM_OC3);
	timer_set_oc_polarity_high(TIM1, TIM_OC4);
	timer_enable_oc_output(TIM1, TIM_OC1);
	timer_enable_oc_output(TIM1, TIM_OC2);
	timer_enable_oc_output(TIM1, TIM_OC3);
	timer_enable_oc_output(TIM1, TIM_OC4);
	timer_enable_break_main_output(TIM1); // Required for Timer 1 and 8 PWM output to work

	timer_generate_event(TIM1, TIM_EGR_UG);

	timer_enable_counter(TIM1);
}

static void setup_timer3_pwm(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_output_options(GPIOC, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO6 | GPIO7 | GPIO8 | GPIO9);
	gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7 | GPIO8 | GPIO9);

	gpio_set_af(GPIOC, GPIO_AF2, GPIO6 | GPIO7 | GPIO8 | GPIO9);

	timer_reset(TIM3);
	rcc_periph_clock_enable(RCC_TIM3);

	timer_continuous_mode(TIM3);

	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
	           TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);

	timer_set_period(TIM3, 64000);
	timer_set_prescaler(TIM3, 19); // for 50 Hz, this needs to be 1 for 500 Hz output

	timer_set_oc_value(TIM3, TIM_OC1, 0);
	timer_set_oc_value(TIM3, TIM_OC2, 0);
	timer_set_oc_value(TIM3, TIM_OC3, 0);
	timer_set_oc_value(TIM3, TIM_OC4, 0);

	timer_enable_preload(TIM3);

	timer_set_oc_polarity_high(TIM3, TIM_OC1);
	timer_set_oc_polarity_high(TIM3, TIM_OC2);
	timer_set_oc_polarity_high(TIM3, TIM_OC3);
	timer_set_oc_polarity_high(TIM3, TIM_OC4);
	timer_enable_oc_output(TIM3, TIM_OC1);
	timer_enable_oc_output(TIM3, TIM_OC2);
	timer_enable_oc_output(TIM3, TIM_OC3);
	timer_enable_oc_output(TIM3, TIM_OC4);
	timer_enable_break_main_output(TIM3); // Required for Timer 1 and 8 PWM output to work

	timer_generate_event(TIM3, TIM_EGR_UG);

	timer_enable_counter(TIM3);
}

static void setup_timer2_pwm(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_set_output_options(GPIOD, GPIO_OTYPE_PP,
	                    GPIO_OSPEED_50MHZ, GPIO3 | GPIO4 | GPIO6 | GPIO7);
	gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3 | GPIO4 | GPIO6 | GPIO7);

	gpio_set_af(GPIOD, GPIO_AF2, GPIO3 | GPIO4 | GPIO6 | GPIO7);

	timer_reset(TIM2);
	rcc_periph_clock_enable(RCC_TIM2);

	timer_continuous_mode(TIM2);

	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
	           TIM_CR1_DIR_UP);
	timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM2, TIM_OC3, TIM_OCM_PWM1);
	timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);

	timer_set_period(TIM2, 64000);
	timer_set_prescaler(TIM2, 19); // for 50 Hz, this needs to be 1 for 500 Hz output

	timer_set_oc_value(TIM2, TIM_OC1, 0);
	timer_set_oc_value(TIM2, TIM_OC2, 0);
	timer_set_oc_value(TIM2, TIM_OC3, 0);
	timer_set_oc_value(TIM2, TIM_OC4, 0);

	timer_enable_preload(TIM2);

	timer_set_oc_polarity_high(TIM2, TIM_OC1);
	timer_set_oc_polarity_high(TIM2, TIM_OC2);
	timer_set_oc_polarity_high(TIM2, TIM_OC3);
	timer_set_oc_polarity_high(TIM2, TIM_OC4);
	timer_enable_oc_output(TIM2, TIM_OC1);
	timer_enable_oc_output(TIM2, TIM_OC2);
	timer_enable_oc_output(TIM2, TIM_OC3);
	timer_enable_oc_output(TIM2, TIM_OC4);
	timer_enable_break_main_output(TIM2); // Required for Timer 1 and 8 PWM output to work

	timer_generate_event(TIM2, TIM_EGR_UG);

	timer_enable_counter(TIM2);
}

// This configures all vehicle PWM channels:
void QuadRotor_PWM_init(void)
{
	/*
		Do hardware-specific things here:
	 */
	setup_timer1_pwm();
	#ifdef HIGHSIDE_SWITCHES_USE_PWM
		setup_timer2_pwm();
	#endif
	setup_timer3_pwm();
}

// Functions to initialize PWM channels correcponding to individual motor/ESC's

void QuadRotor_motor1_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM3, TIM_OC1, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM3, TIM_OC1);
}

void QuadRotor_motor2_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM3, TIM_OC2, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM3, TIM_OC2);
}

void QuadRotor_motor3_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM3, TIM_OC3, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM3, TIM_OC3);
}

void QuadRotor_motor4_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM3, TIM_OC4, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM3, TIM_OC4);
}

void QuadRotor_motor5_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM1, TIM_OC2);
}

void QuadRotor_motor6_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM1, TIM_OC4);
}

void QuadRotor_motor7_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM1, TIM_OC1);
}

void QuadRotor_motor8_start(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MIN_VAL);
	timer_enable_oc_output(TIM1, TIM_OC3);
}

// Functions to stop PWM channels correcponding to individual motor/ESC's
void QuadRotor_motor1_stop(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM3, TIM_OC1, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM3, TIM_OC1);
}
void QuadRotor_motor2_stop(void)
{
	/*
		Do hardware-specific things here:
	 */	
	timer_set_oc_value(TIM3, TIM_OC2, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM3, TIM_OC2);
}
void QuadRotor_motor3_stop(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM3, TIM_OC3, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM3, TIM_OC3);
}
void QuadRotor_motor4_stop(void)
{
	/*
		Do hardware-specific things here:
	 */	
	timer_set_oc_value(TIM3, TIM_OC4, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM3, TIM_OC4);
}

void QuadRotor_motor5_stop(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM1, TIM_OC2, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM1, TIM_OC2);
}
void QuadRotor_motor6_stop(void)
{
	/*
		Do hardware-specific things here:
	 */	
	timer_set_oc_value(TIM1, TIM_OC4, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM1, TIM_OC4);
}
void QuadRotor_motor7_stop(void)
{
	/*
		Do hardware-specific things here:
	 */
	timer_set_oc_value(TIM1, TIM_OC1, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM1, TIM_OC1);
}
void QuadRotor_motor8_stop(void)
{
	/*
		Do hardware-specific things here:
	 */	
	timer_set_oc_value(TIM1, TIM_OC3, TIM_OC_MIN_VAL);
	timer_disable_oc_output(TIM1, TIM_OC3);
}

// Functions to set PWM channels correcponding to individual motor/ESC's:

void QuadRotor_motor1_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM3, TIM_OC1, oc_value_scaled);
	}
}

void QuadRotor_motor2_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM3, TIM_OC2, oc_value_scaled);
	}
}

void QuadRotor_motor3_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM3, TIM_OC3, oc_value_scaled);
	}
}

void QuadRotor_motor4_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM3, TIM_OC4, oc_value_scaled);
	}
}

void QuadRotor_motor5_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM1, TIM_OC2, oc_value_scaled);
	}
}

void QuadRotor_motor6_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM1, TIM_OC4, oc_value_scaled);
	}
}

void QuadRotor_motor7_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM1, TIM_OC1, oc_value_scaled);
	}
}

void QuadRotor_motor8_setDuty(float duty)
{
	/*
		Do hardware-specific things here:
	 */
	uint32_t oc_value_scaled = (uint32_t)((float)(TIM_OC_MAX_VAL-TIM_OC_MIN_VAL)*duty + (float)TIM_OC_MIN_VAL);
	if(duty > MIN_DUTY && duty <= MAX_DUTY)
	{
		timer_set_oc_value(TIM1, TIM_OC3, oc_value_scaled);
	}
}
