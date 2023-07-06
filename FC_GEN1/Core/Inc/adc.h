/*
 * adc.h
 *
 *  Created on: Oct 30, 2022
 *      Authors: Samuel Parent,
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

/*----------INCLUDES----------*/

#include <stdint.h>
#include <main.h>

/*----------MACROS------------*/

#define TOTAL_ADC_MODULES 			(3)
#define ADC_CHANNELS_PER_MODULE 	(16)
#define NUM_ADC_BITS 				(12)
#define MISSING_CONVERTER 			(0)

/*----------TYPEDEFS----------*/

// TODO: Eventually add more specific error codes
typedef enum {
	ADC_OK = 1,
	INVALID_ADC_NUM,
	INVALID_NUM_CHANNELS,
	FAIL_ADC_INIT,
	SCALED_ARRAY_DOES_NOT_MATCH_BUFFER_SIZE,
	ADC_READING_FAILED,
	ADC_TIMEOUT_REACHED,
	NO_CONVERSION_TYPE,
	INVALID_CHANNEL_NUMBER,
	DUPLICATE_CHANNELS
}ADC_Ret_et;

typedef enum {
 ADC_3CYCLES = 4,
 ADC_15CYCLES,
 ADC_28CYCLES,
 ADC_56CYCLES,
 ADC_84CYCLES,
 ADC_112CYCLES,
 ADC_144CYCLES,
 ADC_480CYCLES,
}ADC_Sample_Time_et;


// Define function pointer for scaling
//The first parameter is the raw value passed and the second value is the size of the buffer that the raw value originates form
typedef double(*converter)(uint16_t, uint16_t);

typedef struct{
	// channel is the channel number on the ADC module
	uint8_t channel_number;
	// sample_time determines the number of clock cycles to get a single reading
	ADC_Sample_Time_et sample_time;
	// buffer_len determines the size of the data bufferS
	uint16_t buffer_len;
	// buffer is where data will be stored when conversions are done.
	uint16_t* buffer;
	// function pointer to store the users desired conversion
	converter convert;
}ADC_Channel_st;

typedef struct {
	// hadc is a pointer to the adc handle. The handle should be a global variable in the main.c file
	ADC_HandleTypeDef* hadc;
	// adc_number is the module number of the adc being used
	uint8_t adc_num;
	// num_channels determines the number of channels that will be used
	uint8_t num_channels;
	// channels is an array of adc channels, the order in which they are passed determines their rank
	ADC_Channel_st* channels;
}ADC_st;

/*----------PUBLIC FUNCTION DECLARATIONS----------*/
// ADC_Init initializes an ADC module
ADC_Ret_et ADC_Init(ADC_st* adc);
// ADC_Scan starts an ADC scan based on the given configurations
ADC_Ret_et ADC_Scan(ADC_st* adc);

// Get_Single_Chan_Average return an average of the buffers in a channel and returns a uint16_t
uint16_t Get_Single_Chan_Average(ADC_st* adc, uint8_t channel);

// Get_Single_Channel_Average_Scaled scales a single channel average reading depending on the conversion type and returns it as a double
double Get_Single_Chan_Average_Scaled(ADC_st* adc, uint8_t channel);

// Get_All_Chan_Averages fills the averages array with the averages of each channel (in order passed to init funciton)
ADC_Ret_et Get_Chan_Averages(ADC_st* adc, uint16_t averages[], uint16_t size);

// Get_All_Chan_Averages_Scaled fills the averages array with the scaled averages of each channel (in order passed to init funciton)
ADC_Ret_et Get_Chan_Averages_Scaled(ADC_st* adc, double averages[], uint16_t size);

// Define scaling functions
double Get_Voltage_Conversion(uint16_t raw, uint16_t size);

#endif /* INC_ADC_H_ */
