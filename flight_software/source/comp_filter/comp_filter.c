/*
 * File: comp_filter.c
 *
 * Author: Abhimanyu Ghosh
 * 			Controls and Robotics Research Laboratory (CRRL)
 * 			NYU Polytechnic School of Engineering
 * 			(c) 2014-2015
 */

#include "comp_filter.h"

volatile lpf_data_struct acc_x_lpf;	// Low-pass filter data structure for X-axis accelerometer
volatile lpf_data_struct acc_y_lpf;	// Low-pass filter data structure for Y-axis accelerometer
volatile lpf_data_struct acc_z_lpf;	// Low-pass filter data structure for Z-axis accelerometer
volatile hpf_data_struct acc_z_hpf;	// For lulz..

volatile hpf_data_struct gyr_pitch_hpf;	// High-pass filter data structure for X-axis gyroscope
volatile hpf_data_struct gyr_roll_hpf;	// High-pass filter data structure for Y-axis gyroscope

volatile lpf_data_struct gyr_pitch_lpf;	// Low-pass filter data structure for X-axis gyroscope
volatile lpf_data_struct gyr_roll_lpf;	// Low-pass filter data structure for Y-axis gyroscope

volatile lpf_data_struct magnetometer_x_lpf;
volatile lpf_data_struct magnetometer_y_lpf;
volatile lpf_data_struct magnetometer_z_lpf;

volatile vehicle_bias_data_struct quadrotor_imu_bias_values;

/*
 * Initializes low and high-pass filter data structures for accelerometer and gyroscope data streams
 * Also initializes vehicle filtered state variables to appropriate starting values.
 *
 * Behavior: This function does not block, use recursion or return anything.
 */

void init_comp_filter(filtered_quadrotor_state* stvar)
{
	init_lpf_variables(&acc_x_lpf, ACCEL_X_LPF_CUTOFF_HERTZ);
	init_lpf_variables(&acc_y_lpf, ACCEL_Y_LPF_CUTOFF_HERTZ);

	init_lpf_variables(&acc_z_lpf, ACCEL_Z_LPF_CUTOFF_HERTZ);

	init_hpf_variables(&acc_z_hpf, ACCEL_Z_HPF_CUTOFF_HERTZ);

	init_hpf_variables(&gyr_pitch_hpf, GYRO_HPF_CUTOFF_HERTZ);
	init_hpf_variables(&gyr_roll_hpf, GYRO_HPF_CUTOFF_HERTZ);

	init_lpf_variables(&gyr_pitch_lpf, GYRO_LPF_CUTOFF_HERTZ);
	init_lpf_variables(&gyr_roll_lpf, GYRO_LPF_CUTOFF_HERTZ);

	init_lpf_variables(&magnetometer_x_lpf, MAGNETOMETER_LPF_CUTOFF);
	init_lpf_variables(&magnetometer_y_lpf, MAGNETOMETER_LPF_CUTOFF);
	init_lpf_variables(&magnetometer_z_lpf, MAGNETOMETER_LPF_CUTOFF);

	stvar->pitch = 0.0f;
	stvar->roll = 0.0f;
	stvar->yaw = 0.0f;

	stvar->vertical_dynamic_acceleration_post_lpf = 0.0f;
	stvar->vertical_velocity = 0.0f;
	stvar->height = 0.0f;

	stvar->x_velocity = 0.0f;
	stvar->y_velocity = 0.0f;

	stvar->x_displacement = 0.0f;
	stvar->y_displacement = 0.0f;
}

void init_lpf_variables(lpf_data_struct* lpf_str, float cutoff_freq_hertz)
{
	lpf_str->prev_output = 0.0f;
	float rc_val = 1.0f/(2.0f*PI_VAL*cutoff_freq_hertz);
	lpf_str->alpha_val = DT_FILTER_LOOP/(DT_FILTER_LOOP+rc_val);
}

/*
 * Standard 1st-order Low-pass filter implementation, algorithmic inspiration from:
 * http://en.wikipedia.org/wiki/Low-pass_filter#Discrete-time_realization
 */

float lowpass_filter(float input, lpf_data_struct* lpf_str)
{
	float output_value = (lpf_str->alpha_val*input) + ((1.0f-lpf_str->alpha_val)*lpf_str->prev_output);
	lpf_str->prev_output = output_value;
	return output_value;
}

void init_hpf_variables(hpf_data_struct* hpf_str, float cutoff_freq_hertz)
{
	hpf_str->prev_input = 0.0f;
	hpf_str->prev_output = 0.0f;
	float rc_val = 1.0f/(2.0f*PI_VAL*cutoff_freq_hertz);
	hpf_str->alpha_val = rc_val/(DT_FILTER_LOOP+rc_val);
}

/*
 * Standard 1st-order High-pass filter implementation, algorithmic inspiration from:
 * http://en.wikipedia.org/wiki/High-pass_filter#Discrete-time_realization
 */

float highpass_filter(float input, hpf_data_struct* hpf_str)
{
	float output_value = (hpf_str->alpha_val*hpf_str->prev_output)
									+ (hpf_str->alpha_val*(input-hpf_str->prev_input));
	hpf_str->prev_input = input;
	hpf_str->prev_output = output_value;
	return output_value;
}

void get_corrected_scaled_gyro_data(imu_scaled_data_struct* input, float* output)
{
	output[AXIS_ROLL] = input->gyro_data[AXIS_ROLL] 	+ quadrotor_imu_bias_values.roll_gyro_bias;
	output[AXIS_PITCH] = input->gyro_data[AXIS_PITCH] 	+ quadrotor_imu_bias_values.pitch_gyro_bias;
	output[AXIS_YAW] = input->gyro_data[AXIS_YAW] 		+ quadrotor_imu_bias_values.yaw_gyro_bias;
}

void get_corrected_scaled_accelerometer_data(imu_scaled_data_struct* input, float* output)
{
	output[AXIS_X] = input->accel_data[AXIS_X] + quadrotor_imu_bias_values.x_accelerometer_bias;
	output[AXIS_Y] = input->accel_data[AXIS_Y] + quadrotor_imu_bias_values.y_accelerometer_bias;
	output[AXIS_Z] = input->accel_data[AXIS_Z] + quadrotor_imu_bias_values.z_accelerometer_bias;
}

static float get_vector_magnitude(float magnetometer_data_input[3])
{
	return sqrtf(magnetometer_data_input[AXIS_X]*magnetometer_data_input[AXIS_X] +
				 magnetometer_data_input[AXIS_Y]*magnetometer_data_input[AXIS_Y] +
				 magnetometer_data_input[AXIS_Z]*magnetometer_data_input[AXIS_Z]);
}

static void normalize_magnetometer_data(imu_scaled_data_struct input, float* normalized_output)
{
	uint8_t i = 0U;
	float magnetometer_data_magnitude = get_vector_magnitude(input.magnetometer_data);
	if(magnetometer_data_magnitude > 0.0f)
	{
		for(i=0U; i<3U; ++i)
		{
			normalized_output[i] = input.magnetometer_data[i]/magnetometer_data_magnitude;	
		}			
	}
	else
	{
		for(i=0U; i<3U; ++i)
		{
			normalized_output[i] = 0.0f;	
		}
	}
}

void get_filtered_vehicle_state(filtered_quadrotor_state* statevar, imu_scaled_data_struct* input)
{
	float corrected_accelerometer_data[3];
	float corrected_gyro_data[3];
	float acc_x_filtered, acc_y_filtered, acc_z_filtered, acc_z_filtered_lpf;
	float gyr_pitch_filtered, gyr_roll_filtered,
			gyr_pitch_filtered_post_hpf, gyr_roll_filtered_post_hpf;
	float roll_accel2, pitch_accel2;
	float normalized_magnetometer_data[3];
	float magnetometer_x_filtered, magnetometer_y_filtered, magnetometer_z_filtered;

	get_corrected_scaled_accelerometer_data(input, corrected_accelerometer_data);
	get_corrected_scaled_gyro_data(input, corrected_gyro_data);

	acc_x_filtered = lowpass_filter(corrected_accelerometer_data[AXIS_X], &acc_x_lpf);//1
	acc_y_filtered = lowpass_filter(corrected_accelerometer_data[AXIS_Y], &acc_y_lpf);//0

	acc_z_filtered_lpf = lowpass_filter(corrected_accelerometer_data[AXIS_Z], &acc_z_lpf);
	acc_z_filtered = acc_z_filtered_lpf;

	normalize_magnetometer_data(*input, normalized_magnetometer_data);

	// magnetometer_x_filtered = lowpass_filter(normalized_magnetometer_data[AXIS_X], &magnetometer_x_lpf);
	// magnetometer_y_filtered = lowpass_filter(normalized_magnetometer_data[AXIS_Y], &magnetometer_y_lpf);
	// magnetometer_z_filtered = lowpass_filter(normalized_magnetometer_data[AXIS_Z], &magnetometer_z_lpf);

	magnetometer_x_filtered = normalized_magnetometer_data[AXIS_X];
	magnetometer_y_filtered = normalized_magnetometer_data[AXIS_Y];
	magnetometer_z_filtered = normalized_magnetometer_data[AXIS_Z];

	/*
	 * Obtain vehicle dynamic acceleration by subtracting filtered Z-axis
	 * accelerometer reading by static acceleration due to gravity (i.e.
	 * 9.810 m/s = 1G)
	 *
	 * TODO: MAKE SURE STATIC ACCELERATION REPORTED HERE IS ACTUALLY 9.810 AND NOT ITS
	 * NEGATIVE!!
	 *
	 * Important note: Using these conventions, dynamic acceleration downward is considered NEGATIVE!!
	 */
	float vertical_dynamic_acceleration = acc_z_filtered - 9.810f;
	statevar->vertical_dynamic_acceleration_post_lpf = vertical_dynamic_acceleration;

#ifdef GYRO_HPF_ENABLED_X
	gyr_pitch_filtered_post_hpf = highpass_filter(corrected_gyro_data[AXIS_PITCH], &gyr_pitch_hpf);
#endif
#ifdef GYRO_HPF_ENABLED_Y
	gyr_roll_filtered_post_hpf = highpass_filter(corrected_gyro_data[AXIS_ROLL], &gyr_roll_hpf);
#endif

#ifdef GYRO_HPF_LPF_ENABLED_X
	gyr_pitch_filtered = lowpass_filter(gyr_pitch_filtered_post_hpf, &gyr_pitch_lpf);
#endif
#ifdef GYRO_HPF_LPF_ENABLED_Y
	gyr_roll_filtered = lowpass_filter(gyr_roll_filtered_post_hpf, &gyr_roll_lpf);
#endif

#ifndef GYRO_HPF_LPF_ENABLED_X
	gyr_pitch_filtered = gyr_pitch_filtered_post_hpf;
#endif
#ifndef GYRO_HPF_LPF_ENABLED_Y
	gyr_roll_filtered = gyr_roll_filtered_post_hpf;
#endif

	// acc_x_filtered = corrected_accelerometer_data[AXIS_X];
	// acc_y_filtered = corrected_accelerometer_data[AXIS_Y];

	gyr_roll_filtered = corrected_gyro_data[AXIS_ROLL];
	gyr_pitch_filtered = corrected_gyro_data[AXIS_PITCH];

	/*
	 * Obtain angular position estimate from accelerometers, and
	 * combine this with estimate obtained from gyro, using Complementary Filter:
	 */

	roll_accel2 = RADIANS_TO_DEGREES_CONVERSION_FACTOR * atan2f(acc_x_filtered,
					sqrtf(acc_y_filtered * acc_y_filtered + acc_z_filtered * acc_z_filtered));

	/*
	 * atan2 and atan2f return in the range [-PI,PI]
	 * We want to check if the return from the atan2 used to compute roll_accel2 is
	 * either positive or negative pi/2 radians (i.e. +/- 90 Degrees)
	 * This would lead to a singularity (essentially gimbal lock)
	 * At that point we opt to reply purely on gyroscope data as we transition to the
	 * next quadrant of operation
	 *
	 * In real-life flight scenarios this should not really be a factor as the implication
	 * would be that the vehicle is rolled over a full +/- 90 Degrees, essentially leading
	 * to loss of all lift unless in the middle of acrobatics, in which case gimbal lock
	 * would be a relatively brief occurence
	 *
	 */

	if(roll_accel2 == PI_VAL * 0.5f || roll_accel2 == -1.0f * PI_VAL * 0.5f)
	{
		/*
		 * We trust only the gyro through a gimbal lock condition:
		 */

		statevar->pitch = statevar->pitch+(gyr_pitch_filtered*DT_FILTER_LOOP);
		statevar->roll = statevar->roll+(gyr_roll_filtered*DT_FILTER_LOOP);
	}
	else // Normal computation of pitch based on Y,Z accelerations and roll angle cosine
	{
		pitch_accel2 = -1.0f * RADIANS_TO_DEGREES_CONVERSION_FACTOR * atan2f(acc_y_filtered/cos(roll_accel2*DEGREES_TO_RADIANS_CONVERSION_FACTOR),acc_z_filtered/cos(roll_accel2*DEGREES_TO_RADIANS_CONVERSION_FACTOR));

		/*
		 * Under normal circumstances (i.e. no gimbal lock), combine this with estimate
		 * obtained from gyro, using Complementary Filter:
		 */

		statevar->pitch = GYRO_WEIGHT*(statevar->pitch+(gyr_pitch_filtered*DT_FILTER_LOOP))+ACCEL_WEIGHT*pitch_accel2;
		statevar->roll = GYRO_WEIGHT*(statevar->roll+(gyr_roll_filtered*DT_FILTER_LOOP))+ACCEL_WEIGHT*roll_accel2;
	}
	/*
		Adapted from https://sites.google.com/site/myimuestimationexperience/sensors/magnetometer

		Heading (or yaw) =atan2((-ymag*cos(Roll) + zmag*sin(Roll)),
		 					(xmag*cos(Pitch) + ymag*sin(Pitch)*sin(Roll)+ zmag*sin(Pitch)*cos(Roll))) 
	 */

	statevar->yaw = RADIANS_TO_DEGREES_CONVERSION_FACTOR *
					atan2f((-1.0f*magnetometer_y_filtered*cos(statevar->roll) + magnetometer_z_filtered*sin(statevar->roll)),
									(magnetometer_x_filtered*cos(statevar->pitch) + 
									magnetometer_y_filtered*sin(statevar->pitch)*sin(statevar->roll) + 
									magnetometer_z_filtered*sin(statevar->pitch)*cos(statevar->roll)) );
}

void do_bias_calculation(imu_scaled_data_struct *imu_data)
{
	uint16_t i = 0U;
	// imu_scaled_data_struct imu_data;

	float roll_gyro_running_sum = 0.0f;
	float pitch_gyro_running_sum = 0.0f;
	float yaw_gyro_running_sum = 0.0f;

	float x_accelerometer_running_sum = 0.0f;
	float y_accelerometer_running_sum = 0.0f;
	float z_accelerometer_running_sum = 0.0f;

	for(i=0U; i<BIAS_CALC_NUM_SAMPLES; ++i)
	{
		get_scaled_imu_data(imu_data);

		roll_gyro_running_sum += imu_data->gyro_data[AXIS_ROLL];
		pitch_gyro_running_sum += imu_data->gyro_data[AXIS_PITCH];
		yaw_gyro_running_sum += imu_data->gyro_data[AXIS_YAW];

		x_accelerometer_running_sum += imu_data->accel_data[AXIS_X];
		y_accelerometer_running_sum += imu_data->accel_data[AXIS_Y];
		z_accelerometer_running_sum += imu_data->accel_data[AXIS_Z];

		timekeeper_delay((uint16_t)BIAS_CALC_SAMPLE_DT_MS);
	}
	quadrotor_imu_bias_values.roll_gyro_bias = (float)GYRO_Y_IDEAL_READING - (roll_gyro_running_sum/(float)BIAS_CALC_NUM_SAMPLES);
	quadrotor_imu_bias_values.pitch_gyro_bias = (float)GYRO_X_IDEAL_READING - (pitch_gyro_running_sum/(float)BIAS_CALC_NUM_SAMPLES);
	quadrotor_imu_bias_values.yaw_gyro_bias = (float)GYRO_Z_IDEAL_READING - (yaw_gyro_running_sum/(float)BIAS_CALC_NUM_SAMPLES);

	quadrotor_imu_bias_values.x_accelerometer_bias = (float)ACCELEROMETER_X_IDEAL_READING - (x_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES);
	quadrotor_imu_bias_values.y_accelerometer_bias = (float)ACCELEROMETER_Y_IDEAL_READING - (y_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES);
	quadrotor_imu_bias_values.z_accelerometer_bias = (float)ACCELEROMETER_Z_IDEAL_READING - (z_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES);

#ifdef DEBUG_BIAS_VALUES
	printf("Accel bias: x: %f y: %f z: %f\r\n", quadrotor_imu_bias_values.x_accelerometer_bias,
												quadrotor_imu_bias_values.y_accelerometer_bias,
												(float)ACCELEROMETER_Z_IDEAL_READING - (z_accelerometer_running_sum/(float)BIAS_CALC_NUM_SAMPLES));//quadrotor_imu_bias_values.z_accelerometer_bias);
	printf("Gyro bias: x: %f y: %f z: %f\r\n", quadrotor_imu_bias_values.roll_gyro_bias,
												quadrotor_imu_bias_values.pitch_gyro_bias,
												quadrotor_imu_bias_values.yaw_gyro_bias);
#endif
}

void propagate_compensated_vehicle_height(float height_vehicle_body_frame, 
											float height_sensor_position_body_frame_x, 
											float height_sensor_position_body_frame_y, 
											filtered_quadrotor_state *st)
{
	float roll_cosine = cos(DEGREES_TO_RADIANS_CONVERSION_FACTOR * st->roll);
	float pitch_cosine = cos(DEGREES_TO_RADIANS_CONVERSION_FACTOR * st->pitch);

	float roll_sine = sin(DEGREES_TO_RADIANS_CONVERSION_FACTOR * st->roll);
	float pitch_sine = sin(DEGREES_TO_RADIANS_CONVERSION_FACTOR * st->pitch);
	// st->height = height_vehicle_body_frame * cos(DEGREES_TO_RADIANS_CONVERSION_FACTOR * st->roll)
	// 									  	* cos(DEGREES_TO_RADIANS_CONVERSION_FACTOR * st->pitch);
	float new_height = 	(-1.0f * height_sensor_position_body_frame_x * pitch_cosine * roll_sine) +
					(height_sensor_position_body_frame_y * pitch_sine) + 
					(height_vehicle_body_frame * roll_cosine * pitch_cosine);
	st->vertical_velocity = (float)(new_height - st->height) / (float)0.035;
	st->height = new_height;
}