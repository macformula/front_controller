/*
 * timers_pwm.h
 *
 *  Created on: September 9, 2022
 *  Author: Samuel Parent
 */

/*
	Advanced Timers:
		tim1
		tim8

	General Purpose Timers:
		tim2
		tim3
		tim4
		tim5
		tim9
		tim10
		tim11
		tim12
		tim13
		tim14

	Basic Timers:
		tim6
		tim7
*/

#ifndef INC_TIMERS_PWM_H_
#define INC_TIMERS_PWM_H_

/*----------INCLUDES----------*/

#include <stdint.h>
#include "main.h"

/*----------MACROS & DEFINES------------*/

#define COUNTING_PERIOD_SLOW	(1000)
#define MAX_TIM_FREQ_SLOW					(1000U)		// Max Freq = 1kHz for any clock speed

#define COUNTING_PERIOD_FAST				(100U)		// Max Freq = Clk_speed / 100 for any clock speed
#define MAX_TIM_FREQ_FAST(CLOCK_FREQ)		((CLOCK_FREQ) / COUNTING_PERIOD_FAST)

#define DEFAULT_IT_FREQ						(0U)
#define DEFAULT_IT_PERIOD					(0U)

#define MAX_DUTY_CYCLE						(100U)		// 100% duty cycle
#define MAX_IT_FREQ 						(1000U)		// This can be modified if needed
#define MAX_REP_COUNTER 					(0xffffU)	// Rep counter is a 16 bit register

/*----------TYPEDEFS----------*/

// Timer can be initialized with period or frequency (freq for higher speed)
typedef enum {
	PERIOD = 1,
	FREQ,
}Timing_Model_et;

// Timer channel Enables
typedef struct{
	//Uses "Bit-fields" for efficient storing
	uint8_t en_ch1 : 1;
	uint8_t en_ch2 : 1;
	uint8_t en_ch3 : 1;
	uint8_t en_ch4 : 1;
	uint8_t en_ch_all : 1;
}Channel_Config_st;

// Configuration for periodic interrupts
typedef struct {
	// Determines if interrupts will be enabled
	uint8_t en_it;
	// Can only be different from timer period if advanced timer (1 or 8), uses repetition counter
	uint32_t period_ms;
	// Can only be different form timer frequency if advanced timer (1 or 8), uses repetition counter
	uint16_t freq_hz;
}Tim_Interrupt_st;

// Timer_Type_et distinguishes timer types as different timers have different capabilities
typedef enum {
	// Timers 1 and 8: 4 PWM channels and repetition counter. Use for PWM + periodic interrupts or one-shot timer.
	ADVANCED_TIMER = 1,
	// Timers 2-5 and 9-14: 1-4 channels. Use for PWM or periodic interrupts or one-shot timer.
	GENERAL_TIMER,
	// Timers 6 and 7: 0 PWM channels. Use for periodic interrupts or one-shot timer only.
	BASIC_TIMER,
}Tim_Type_et;

// Auto-configured: DO NOT WRITE. Timer_Info_st stores info relevant to the specific timer being initialized
typedef struct{
	// Number of PWM channels
	uint8_t num_channels;
	// Determines specific timer capabilities (see Timer_Type_et definition)
	Tim_Type_et tim_type;
	// Set by Timer_Init and unset by Timer_Stop
	uint8_t tim_initialized;
}tim_metadata_st;

// Timer_st stores the user input to timer functions
typedef struct {
	// Pointer to the timer handle being used
	TIM_HandleTypeDef* htim;
	// Timer numbers range from 1 to 14
	uint8_t tim_num;
	// Determines whether timer will be initialized with period or frequency
	Timing_Model_et timing;
	// Period in milliseconds. Minimum period of 1 ms. Use for slower timer
	uint32_t period_ms;
	// Frequency in Hz. Minimum frequency of 1. Max frequency of 160'000 Hz. Use for faster timer
	uint32_t freq_hz;
	// Determines which PWM channels will be enabled
	Channel_Config_st channels;
	// Periodic interrupt configurations
	Tim_Interrupt_st it_config;
	// DO NOT WRITE. Auto-configured. Stores info about timer type and number of channels
	tim_metadata_st __metadata;
}Timer_st;

//TODO: comment
typedef struct {
	// Timer which contains the PWM channel
	Timer_st* tim;
	// PWM channel number
	uint8_t chan_num;
	// Duty cycle is the percentage of the period for which the signal is high
	uint8_t duty;
	// Target duty cycle allows for slow ramp up / slowing down of the PWM
	uint8_t target_duty;
	// Step size will determine how much to move the duty cycle by when the PWM_Move_Towards_Target function is called
	uint8_t duty_step_size;
	// Inverted is a boolean which will make it such that the duty cycle will be (100 - duty)
	uint8_t is_inverted;
}PWM_st;

// TIM_Ret_et shows the status of a timer function
typedef enum {
	// TIM_OK indicates that no error within the function
	TIM_OK = 1,
	// TIM_PERIOD_ZERO indicates the timer period is set to zero and the timing paradigm is PERIOD
	TIM_PERIOD_ZERO,
	// TIM_FREQ_ZERO indicates the timer freq is set to zero and the timing paradigm is FREQUENCY
	TIM_FREQ_ZERO,
	// TIM_NUM_INVALID indicates that the timer number is not within range
	TIM_NUM_INVALID,
	// TIM_FREQ_INVALID indicates that the timer cannot be set to the given frequency
	TIM_FREQ_INVALID,
	// TIM_IT_INVALID_FREQ indicates that the timer interrupts cannot be initialized with the given freq
	TIM_IT_INVALID_FREQ,
	// TIM_IT_INVALID_PERIOD indicates that the timer interrupts cannot be initialized with the given period
	TIM_IT_INVALID_PERIOD,
	// TIM_RC_OVERFLOW indicate that the Repetition Counter cannot be set as it would exceed its maximum
	TIM_RC_OVERFLOW,
	// TIM_NO_RC indicates that the given timer does not have a Repetition Counter
	TIM_NO_RC,
	// TIM_PWM_CH_UNSUPPORTED indicates that one of the enabled pwm channels cannot be supported
	TIM_PWM_CH_UNSUPPORTED,
	// TIM_PWM_UNSUPPORTED indicates that pwm generation is not supported on given timer
	TIM_PWM_UNSUPPORTED,
	// TIM_PWM_INIT_FAIL indicates that the function "HAL_TIM_PWM_Init" failed
	TIM_PWM_INIT_FAIL,
	// TIM_BASE_INTI_FAIL indicates that the function "HAL_TIM_Base_Init" failed
	TIM_BASE_INIT_FAIL,
	// TIM_CONFIG_CLK_FAIL indicates that the function "HAL_TIM_ConfigClockSource" failed
	TIM_CONFIG_CLK_FAIL,
	// TIM_MASTER_CONFIG_FAIL indicates that the function "HAL_TIMEx_MasterConfigSynchronization" failed
	TIM_MASTER_CONFIG_FAIL,
	// TIM_CONFIG_DEADTIME_FAIL indicates that the function "HAL_TIMEx_ConfigBreakDeadTime" failed
	TIM_CONFIG_DEADTIME_FAIL,
	// TIM_PWM_CONFIG_CH_FAIL indicates that the function "HAL_TIM_PWM_ConfigChannel" failed
	TIM_PWM_CONFIG_CH_FAIL,
	// TIM_BASE_START_IT_FAIL indicates that the function "HAL_TIM_Base_Start_IT" failed
	TIM_BASE_START_IT_FAIL,
	// TIM_BASE_DEINIT_FAIL indicates that the function "HAL_TIM_Base_DeInit" failed
	TIM_BASE_DEINIT_FAIL,
	// TIM_ERROR is a general error it does not indicate anything other than it is not PMW_OK
	TIM_ERROR,
}TIM_Ret_et;

// PWM_Ret_et shows the status of a pwm function
typedef enum {
	// PWM_OK indicates that no error within the function
	PWM_OK = 1,
	// PWM_CH_UNSUPPORTED indicates that the given channel is not supported by given timer
	PWM_CH_UNSUPPORTED,
	// PWM_CH_NOT_ENABLED indicates that the given channel is not enabled at the timer level
	PWM_CH_NOT_ENABLED,
	// PWM_INVALID_DUTY indicates that the passed in duty cycle is not within the 0-100% range
	PWM_INVALID_DUTY,
	// PWM_INVALID_TRGT_DUTY indicates that the target duty cycle is not within the 0-100% range
	PWM_INVALID_TRGT_DUTY,
	// PWM_INVALID_CH_NUM indicates that the channel number passed is not within channel limits (1-4)
	PWM_INVALID_CH_NUM,
	// PWM_TIM_UNINIT indicates that the associated timer is not initialized
	PWM_TIM_UNINIT,
	// TIM_BASE_START_IT_FAIL indicates that the function "HAL_TIM_PWM_Start" failed
	PWM_START_FAIL,
	// PWM_STOP_FAIL indicates that the function "HAL_TIM_PWM_Stop" failed
	PWM_STOP_FAIL,
	// PWM_ERROR is a general error it does not indicate anything other than it is not PMW_OK
	PWM_ERROR,
}PWM_Ret_et;

/*----------PUBLIC FUNCTION DECLARATIONS----------*/

TIM_Ret_et Timer_Init(Timer_st* tim);
TIM_Ret_et Timer_Stop(Timer_st* tim);
PWM_Ret_et PWM_Init(PWM_st* pwm);
PWM_Ret_et PWM_Move_Towards_Target(PWM_st* pwm);
PWM_Ret_et PWM_Update_Target(PWM_st* pwm, uint8_t new_target);
PWM_Ret_et PWM_Stop(PWM_st* pwm);

#endif /* INC_TIMERS_PWM_H_ */
