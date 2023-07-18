/**
 ********************************************************************************
 * @file    projectMain.h
 * @author  Samuel Parent
 * @date    2023-07-16
 * @brief   
 ********************************************************************************
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/************************************
 * INCLUDES
 ************************************/
#include "main.h"
#include "rtscheduler.h"
#include "canal_types.h"
#include "canal.h"
#include "adc_lib.h"
#include "uart_lib.h"
#include "timers_pwm.h"
#include "canal_messages.h"
#include "printf.h"
#include "controller_autogen.h"

/************************************
 * MACROS AND DEFINES
 ************************************/
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
/************************************
 * TYPEDEFS
 ************************************/

/************************************
 * EXPORTED VARIABLES
 ************************************/
extern TsCanAL pt1_can;
extern TsCanAL pt2_can;
extern TsCanAL veh_can;
extern Timer_st tim2;
extern PWM_st status_led_pwm;
extern ADC_Channel_st adc1_channels[ADC1_NUM_CHANNELS];
extern ADC_st adc1;
extern UART_st debug_uart;

extern uint8_t driverSpeakerEn; 
extern uint8_t brakeLightEn;
extern uint8_t statusLightDutyCycle; 
extern uint16_t statusLightFreq;

/************************************
 * GLOBAL FUNCTION PROTOTYPES
 ************************************/
void readFromAdc(void);
void setControlSystemInputs(void);
void getControlSystemOutputs(void);
void transmitToAMKMotors(void);
void transmitToBMS(void);
void setDigitalOutputs(void);
void setPWMOutputs(void);
void projectMain();

#ifdef __cplusplus
}
#endif
