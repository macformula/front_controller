/*
 * globals.h
 *
 *  Created on: May 21, 2023
 *      Author: Samuel Parent
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

/*********************************************************
*                       INCLUDES
*********************************************************/

#include "stdint.h"
#include "adc.h"
#include "canal.h"
#include "uart.h"
#include "timers_pwm.h"
#include <pthread.h>

/*********************************************************
*                       DEFINES
*********************************************************/

#define DEBUG_MODE 1

// ADC:
#define NUM_ADC_CHANNELS 					(4)
#define ADC_ROLLING_AVG_WIN_SIZE 			(10)

#define ADC1_NUM_CHANNELS 					(4)

#define APPS1_ADC_CHANNEL 					(10)
#define APPS1_BUFFER_LEN 					(1)
#define APPS1_IDX							(0)

#define APPS2_ADC_CHANNEL 					(11)
#define APPS2_BUFFER_LEN 					(1)
#define APPS2_IDX							(1)

#define STEERING_ANGLE_SENSOR_ADC_CHANNEL 	(12)
#define STEERING_ANGLE_SENSOR_BUFFER_LEN 	(1)
#define STEERING_ANGLE_SENSOR_IDX 			(2)

#define BPPS_BUFFERED_ADC_CHANNEL			(13)
#define BPPS_BUFFERED_BUFFER_LEN 			(1)
#define BPPS_BUFFERED_IDX					(3)

// CAN:
#define PT1_CAN_INST 						(CANAL_INST_CAN_1)
#define PT1_CAN_BAUD						(CANAL_BAUD_500K)
#if DEBUG_MODE
#define PT1_CAN_MODE						(CANAL_MODE_SILENT_LOOPBACK)
#else
#define PT1_CAN_MODE						(CANAL_MODE_NORMAL)
#endif // DEBUG_MODE

#define PT2_CAN_INST						(CANAL_INST_CAN_2)
#define PT2_CAN_BAUD						(CANAL_BAUD_500K)
#define PT2_CAN_MODE						(CANAL_MODE_SILENT_LOOPBACK)

#define VEH_CAN_INST						(CANAL_INST_CAN_3)
#define VEH_CAN_BAUD						(CANAL_BAUD_500K)
#if DEBUG_MODE
#define VEH_CAN_MODE						(CANAL_MODE_SILENT_LOOPBACK)
#else
#define VEH_CAN_MODE						(CANAL_MODE_NORMAL)
#endif // DEBUG_MODE


extern TsCanAL pt1_can;
extern TsCanAL pt2_can;
extern TsCanAL veh_can;
extern ADC_st adc1;
extern Timer_st tim2;
extern PWM_st status_led_pwm;
extern uint16_t adc_values[NUM_ADC_CHANNELS];

/*********************************************************
*                        ADC
*********************************************************/

ADC_HandleTypeDef hadc1;

uint16_t apps1_buffer[APPS1_BUFFER_LEN] = {0};
uint16_t apps2_buffer[APPS2_BUFFER_LEN] = {0};
uint16_t steering_angle_sensor_buffer[STEERING_ANGLE_SENSOR_BUFFER_LEN]= {0};
uint16_t bpps_buffered_buffer[BPPS_BUFFERED_BUFFER_LEN] = {0};

ADC_Channel_st adc1_channels[ADC1_NUM_CHANNELS] = {
	{
		.channel_number = APPS1_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = APPS1_BUFFER_LEN,
		.buffer = apps1_buffer,
		.convert = NULL,
	},
	{
		.channel_number = APPS2_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = APPS2_BUFFER_LEN,
		.buffer = apps2_buffer,
		.convert = NULL,
	},
	{
		.channel_number = STEERING_ANGLE_SENSOR_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = STEERING_ANGLE_SENSOR_BUFFER_LEN,
		.buffer = steering_angle_sensor_buffer,
		.convert = NULL,
	},
	{
		.channel_number = BPPS_BUFFERED_ADC_CHANNEL,
		.sample_time = ADC_15CYCLES,
		.buffer_len = BPPS_BUFFERED_BUFFER_LEN,
		.buffer = bpps_buffered_buffer,
		.convert = NULL,
	},
};

ADC_st adc1 = {
	.hadc = &hadc1,
	.adc_num = 1,
	.num_channels = ADC1_NUM_CHANNELS,
	.channels = adc1_channels,
};

/*********************************************************
*                        CAN
*********************************************************/

extern pthread_mutex_t can_lock;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;
CAN_HandleTypeDef hcan3;

TsCanAL pt1_can = {
	.hcan = &hcan1,
	.canNum = PT1_CAN_INST,
	.baud = PT1_CAN_BAUD,
	.mode = PT1_CAN_MODE,
};

TsCanAL pt2_can = {
	.hcan = &hcan2,
	.canNum = PT2_CAN_INST,
	.baud = PT2_CAN_BAUD,
	.mode = PT2_CAN_MODE,
};

TsCanAL veh_can = {
	.hcan = &hcan3,
	.canNum = VEH_CAN_INST,
	.baud = VEH_CAN_BAUD,
	.mode = VEH_CAN_MODE,
};

/*********************************************************
*                    TIMERS & PWM
*********************************************************/

TIM_HandleTypeDef htim2;

Timer_st tim2 = {
		.htim = &htim2,
		.tim_num = 2,
		.channels = {0,0,0,1},
		.timing = FREQ,
		.freq_hz = 1,
		.it_config = {0},
};
PWM_st status_led_pwm = {
		.tim = &tim2,
		.chan_num = 4,
		.duty = 0,
		.is_inverted = 0,
};

/*********************************************************
*                        UART
*********************************************************/

// NOT USING THE GPS YET (uart2)


/*********************************************************
*                        SPI
*********************************************************/

// NOT USING THE IMU YET
SPI_HandleTypeDef hspi4;


#endif /* INC_GLOBALS_H_ */
