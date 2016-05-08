#ifndef QUADROTOR_PWM_HAL_H
#define QUADROTOR_PWM_HAL_H 1

#include <stdint.h>

#include <hal_common_includes.h>
#include <robot_config.h>

#define MIN_DUTY 0.0f
#define MAX_DUTY 1.0f

#define TIM_OC_MIN_VAL 	0    //33920
#define TIM_OC_MAX_VAL	64000//59520

// This configures all vehicle PWM channels:
void QuadRotor_PWM_init(void);

// Functions to initialize PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_start(void);
void QuadRotor_motor2_start(void);
void QuadRotor_motor3_start(void);
void QuadRotor_motor4_start(void);

void QuadRotor_motor5_start(void);
void QuadRotor_motor6_start(void);
void QuadRotor_motor7_start(void);
void QuadRotor_motor8_start(void);

// Functions to stop PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_stop(void);
void QuadRotor_motor2_stop(void);
void QuadRotor_motor3_stop(void);
void QuadRotor_motor4_stop(void);

void QuadRotor_motor5_stop(void);
void QuadRotor_motor6_stop(void);
void QuadRotor_motor7_stop(void);
void QuadRotor_motor8_stop(void);

// Functions to set PWM channels corresponding to individual motor/ESC's
void QuadRotor_motor1_setDuty(float duty);
void QuadRotor_motor2_setDuty(float duty);
void QuadRotor_motor3_setDuty(float duty);
void QuadRotor_motor4_setDuty(float duty);

void QuadRotor_motor5_setDuty(float duty);
void QuadRotor_motor6_setDuty(float duty);
void QuadRotor_motor7_setDuty(float duty);
void QuadRotor_motor8_setDuty(float duty);

#endif