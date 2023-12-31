/*
 * adc.c
 *
 *  Created on: Oct 30, 2022
 *      Authors: Samuel Parent,
 */

/*----------INCLUDES----------*/

#include "adc.h"

/*----------PRIVATE FUNCTION DEFINITIONS----------*/

#define FILL_BY_CHANNEL 1

/*
 * NOTES:	This is where all of your private helper functions go.
 * 			They will only be visible in this file.
 * 			They do not require a function declaration.
 * 			They should all have the static keyword in front of the prototype.
 *
 * 			Example:
 * 			static uint8_t get_channel(parameters){...}
 */
static void ADC_Channel_Select(uint8_t channel,
		ADC_ChannelConfTypeDef *ChanConfig) {
	switch (channel) {
	case (0):
		ChanConfig->Channel = ADC_CHANNEL_0;
		return;
	case (1):
		ChanConfig->Channel = ADC_CHANNEL_1;
		return;
	case (2):
		ChanConfig->Channel = ADC_CHANNEL_2;
		return;
	case (3):
		ChanConfig->Channel = ADC_CHANNEL_3;
		return;
	case (4):
		ChanConfig->Channel = ADC_CHANNEL_4;
		return;
	case (5):
		ChanConfig->Channel = ADC_CHANNEL_5;
		return;
	case (6):
		ChanConfig->Channel = ADC_CHANNEL_6;
		return;
	case (7):
		ChanConfig->Channel = ADC_CHANNEL_7;
		return;
	case (8):
		ChanConfig->Channel = ADC_CHANNEL_8;
		return;
	case (9):
		ChanConfig->Channel = ADC_CHANNEL_9;
		return;
	case (10):
		ChanConfig->Channel = ADC_CHANNEL_10;
		return;
	case (11):
		ChanConfig->Channel = ADC_CHANNEL_11;
		return;
	case (12):
		ChanConfig->Channel = ADC_CHANNEL_12;
		return;
	case (13):
		ChanConfig->Channel = ADC_CHANNEL_13;
		return;
	case (14):
		ChanConfig->Channel = ADC_CHANNEL_14;
		return;
	case (15):
		ChanConfig->Channel = ADC_CHANNEL_15;
		return;
	case (16):
		ChanConfig->Channel = ADC_CHANNEL_16;
		return;
	case (17):
		ChanConfig->Channel = ADC_CHANNEL_17;
		return;
	case (18):
		ChanConfig->Channel = ADC_CHANNEL_18;
		return;
	}
}
static void ADC_Chan_Config(ADC_HandleTypeDef *hadc, uint8_t channel) {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	ADC_Channel_Select(channel, &sConfig);
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;

	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

// adc_select sets correct adc instance
static ADC_Ret_et adc_instance_select(ADC_HandleTypeDef *hadc,
		uint8_t adc_number) {
	switch (adc_number) {
	case (1):
		hadc->Instance = ADC1;
		break;
	case (2):
		hadc->Instance = ADC2;
		break;
	case (3):
		hadc->Instance = ADC3;
		break;
	default:
		return INVALID_ADC_NUM;
	}

	return ADC_OK;
}

// adc_set_num_conversions sets and checks the number of conversions that an adc module will have to make
static ADC_Ret_et adc_set_num_conversions(ADC_HandleTypeDef *hadc,
		uint8_t num_channels) {
	// Make sure the number of channels is valid
	if (num_channels <= 0 || num_channels > ADC_CHANNELS_PER_MODULE) {
		return INVALID_NUM_CHANNELS;
	}

	// More than one channel enable scan conversion
	if (num_channels >= 1) {
		hadc->Init.ScanConvMode = ENABLE;
	}

	hadc->Init.NbrOfConversion = num_channels;

	return ADC_OK;
}

// These are the configurations that we leave as is from the autogenerated code
static void adc_default_configs(ADC_HandleTypeDef *hadc) {
	hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc->Init.Resolution = ADC_RESOLUTION_12B;
	hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;

	hadc->Init.ContinuousConvMode = DISABLE;
	hadc->Init.DiscontinuousConvMode = DISABLE;

	hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;

	hadc->Init.DMAContinuousRequests = DISABLE;
	hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
}

/*----------PUBLIC FUNCTION DEFINITIONS----------*/

// ADC_Init initialized an ADC module
ADC_Ret_et ADC_Init(ADC_st *adc) {
	ADC_Ret_et ret;

	// Sets the correct adc instance
	ret = adc_instance_select(adc->hadc, adc->adc_num);
	if (ret != ADC_OK) {
		return ret;
	}

	// Sets the number of conversions (unsure if this function is even needed)
	ret = adc_set_num_conversions(adc->hadc, adc->num_channels);
	if (ret != ADC_OK) {
		return ret;
	}

	// TODO: Make sure all channels passed are valid
	int check[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //to keep track of how many channels use each number
	for (int m = 0; m < adc->num_channels; m++){
			ADC_Channel_st channel = adc->channels[m];
			// check for valid channel #
			if (channel.channel_number > 16 || channel.channel_number < 0) return INVALID_CHANNEL_NUMBER;
			check[channel.channel_number-1] = check[channel.channel_number-1] + 1; //count number of channels using that number
	}

	for (int m = 0; m < adc->num_channels; m++){ //check through array that kept track of how many channels are using each number (only should be 1 max)
		if (check[m] >=2) return DUPLICATE_CHANNELS;
	}


	if (HAL_ADC_Init(adc->hadc) != HAL_OK) {
		return FAIL_ADC_INIT;
	}

	// Return enum type for the case that ADC_OK is true for debugging purposes
	return ret;
}

// ADC_Scan starts an ADC scan based on the given configurations
ADC_Ret_et ADC_Scan(ADC_st *adc) { // This scan function will loop through the channels, taking one reading per channel until all the channels buffers are full
#if FILL_BY_CHANNEL
	for (uint8_t i = 0; i < adc->num_channels; i++) {
		ADC_Chan_Config(adc->hadc, adc->channels[i].channel_number);

		for (uint16_t j = 0; j < adc->channels[i].buffer_len; j++) {
			HAL_Delay(1);
			HAL_ADC_Start(adc->hadc);

			HAL_StatusTypeDef err = HAL_ADC_PollForConversion(adc->hadc, 1000);
			if (err == HAL_TIMEOUT)
				return ADC_TIMEOUT_REACHED;
			if (err != HAL_OK)
				return ADC_READING_FAILED;

			adc->channels[i].buffer[j] = (uint16_t)HAL_ADC_GetValue(adc->hadc);
			HAL_ADC_Stop(adc->hadc);
		}
	}
#else
	uint8_t continue_reading = 1;
	uint16_t reading_idx = 0;

	// fill buffers
	while (continue_reading) {
		// only continue reading if at least one channel is still reading
		continue_reading = 0;

		for (int j = 0; j < adc->num_channels; j++) {
			if (reading_idx < (adc->channels[j].buffer_len)) {
				// record reading to the buffer
				ADC_Chan_Config(adc->hadc, adc->channels[j].channel_number);

				HAL_ADC_Start(adc->hadc);
				HAL_StatusTypeDef err = HAL_ADC_PollForConversion(adc->hadc, 1000);
				if (err == HAL_TIMEOUT)
					return ADC_TIMEOUT_REACHED;
				if (err != HAL_OK)
					return ADC_READING_FAILED;

				if (j == 1) {
					j = j;
				}

				adc->channels[j].buffer[reading_idx] = (uint16_t)HAL_ADC_GetValue(adc->hadc);
				HAL_ADC_Stop(adc->hadc);

				continue_reading = 1;
			}
		}

		reading_idx += 1;
	}
#endif // FILL_BY_CHANNEL
	return ADC_OK;
}

// Get_Single_Chan_Average returns the average reading of a channels buffer
uint16_t Get_Single_Chan_Average(ADC_st *adc, uint8_t channel) {

	uint64_t total = 0;
	uint16_t buff_len;
	uint16_t *buff;

	// find the location of the channel we want to avg
	for (uint8_t i = 0; i < (adc->num_channels); i++) {

		// Loop through the channels to find the desired channel
		if (adc->channels[i].channel_number == channel) {

			//Get the length and values
			buff_len = adc->channels[i].buffer_len;
			buff = adc->channels[i].buffer;

			//Sum of values
			for (int i = 0; i < buff_len; i++) {
				total += buff[i];
			}

			return total / buff_len;
		}
	}
}

double Get_Single_Chan_Average_Scaled(ADC_st *adc, uint8_t channel) {

	uint16_t total = 0;
	uint16_t buff_len;
	uint16_t *buff;
	double average_conversion;

	// find the location of the channel we want to avg
	for (uint8_t i = 0; i < (adc->num_channels); i++) {

		// Loop through the channels to find the desired channel
		if (adc->channels[i].channel_number == channel) {
			if (adc->channels[i].convert != MISSING_CONVERTER) {
				//Get the length and values
				buff_len = adc->channels[i].buffer_len;
				buff = adc->channels[i].buffer;

				//Sum of values
				for (int i = 0; i < buff_len; i++) {
					total = total + buff[i];
				}

				average_conversion = adc->channels[i].convert(
						(total / buff_len), buff_len);
			}
		}
	}
	return average_conversion;
}

// Scale_Buffer fills an array passed by reference with scaled readings based on the function specified in the channel struct
ADC_Ret_et Scale_Buffer(ADC_st *adc, int channel_number, double scaled[],
		int scaled_size) {
	if (adc->channels[channel_number].convert != MISSING_CONVERTER) {
		if (scaled_size != adc->channels[channel_number].buffer_len)
			return SCALED_ARRAY_DOES_NOT_MATCH_BUFFER_SIZE;
		for (int i = 0; i < scaled_size; i++) {
			scaled[i] = adc->channels[channel_number].convert(
					adc->channels[channel_number].buffer[i], scaled_size); // use conversion in channel to scale
		}
	} else {
		return NO_CONVERSION_TYPE;
	}
	return ADC_OK;

}

// Get_All_Chan_Averages fills the averages array with the averages of each channel (in order passed to init funciton)
ADC_Ret_et Get_Chan_Averages(ADC_st *adc, uint16_t averages[], uint16_t size) {
	//TODO: use the get single chan averages to fill the averages array
	uint8_t number_of_channels = adc->num_channels;
	ADC_Channel_st *channel_array = adc->channels;

	// Check that the user defined number of channels is valid
	// Cannot check using sizeof(averages) because it decays to a pointer
	if (size <= 0 || size > number_of_channels) {
		return INVALID_NUM_CHANNELS;
	}

	// Loop through every channel and get its single channel average using the corresponding channel number
	for (int i = 0; i < number_of_channels; i++) {
		uint16_t channel_number = channel_array[i].channel_number;
		averages[i] = Get_Single_Chan_Average(adc, channel_number);
	}

	return ADC_OK;
}

// Get_All_Chan_Averages fills the averages array with the averages of each channel (in order passed to init funciton)
ADC_Ret_et Get_Chan_Averages_Scaled(ADC_st *adc, double averages[],
		uint16_t size) {
	//TODO: use the get single chan averages to fill the averages array
	uint8_t number_of_channels = adc->num_channels;
	ADC_Channel_st *channel_array = adc->channels;

	// Check that the user defined number of channels is valid
	// Cannot check using sizeof(averages) because it decays to a pointer
	if (size <= 0 || size > number_of_channels) {
		return INVALID_NUM_CHANNELS;
	}

	// Loop through every channel and get its single channel average using the corresponding channel number
	for (int i = 0; i < number_of_channels; i++) {
		uint16_t channel_number = channel_array[i].channel_number;
		averages[i] = Get_Single_Chan_Average_Scaled(adc, channel_number);
	}

	return ADC_OK;
}

// ---------- Function Pointers ---------- //
// Convert channel ADC readings to voltages
double Get_Voltage_Conversion(uint16_t raw, uint16_t size) {
	double voltage_conversion;
	//voltage_conversion = raw / (pow(2,NUM_ADC_BITS) / 3.3); //FOR TESTING PURPOSES
	/*
	 * User implementation here
	 */
	return voltage_conversion;
}
