/*
 * canal_fc_messages.c
 *
 *  WARNING: DO NOT EDIT. THIS WAS AN AUTOGENERATED FILE.
 *
 *  Created on: 2023/6/4 
 *     Authors: Samuel Parent, Dharak Verma
 *
 *                    ..::^~~~!!~~~^^:..
 *                .:^!7??JJJJJJJJJJJJ??7!~:.
 *              :~7?JJJ???????????????JJJJ??!^.
 *           .^7?JJJ???JJJJJJ??????????????JJJ?~:
 *          ^7JJ???????777777???JJJJJJJ???????JJ?~.
 *        .!JJ???????????7!~^::::^~!!7??JJ???????J7:
 *       :7JJJ??????????JJJJ??7~:    ..:~7?J??????J?^
 *      .7J???????????????????JJ?!.       :7J??????J?^
 *     .~J??????????????????JJJJ?!.       .7J???????J7.
 *     :?J?????????????JJJJ??7!^.      .:~???????????J~
 *     ^???????????JJJ??7~^:..      .:~7?JJ??????????J!.
 *     ^????????JJ?7~^:.        .:~!?JJJ?????????????J!.
 *     ^????????!^.          .:~7?JJJ????????????????J!.
 *     .7J????!.            :7?JJ?????????????????????^
 *      ~JJ?J7.             ~JJJJ???????????????????J!.
 *      .!J???^.            .^!7?JJJJJJ????????????J7:
 *       .!?J??7^.              .:^~!7???JJJJJ????J7:
 *        .^?JJJJ?7~.                 ..:^~~!7????!.
 *          .!?JJJJ?:                         ..::
 *            :~??!:
 *              ..
 */
 

 /*********************************************************
 *                      INCLUDES
 *********************************************************/
#include <canal_messages.h>
#include <stdint.h>

/*********************************************************
 *                       MACROS
 *********************************************************/
#define CANAL_DATA_BITS (64U)

// CANAL_BIT_MASK will output a mask with (__length__) number of 1's
// e.g.: CANAL_BIT_MASK(5) = 0b0001'1111
#define CANAL_BIT_MASK(__length__) 	(UINT64_MAX >> (CANAL_DATA_BITS - __length__))
    
// CANAL_BIT_INDEX_BE will take a bit index and convert it to work with big endian data
#define CANAL_BIT_INDEX_BE(__idx__) 	((7 - ((__idx__) % 8)) + (((__idx__) / 8) * 8))

// CANAL_CLEAR_BITS will output the contents of (__data__) with bits from (__start__)
// to (__start__+__length__) set to 0
#define CANAL_CLEAR_BITS(__data__, __start__, __length__) 	(~((CANAL_BIT_MASK(__length__) << __start__)) & (__data__))

// CANAL_GET_BITS will output the bits from (__data__) at bit positions (__start__)
// to (__start__+__length__)
#define CANAL_GET_BITS(__data__, __start__, __length__) 	((__data__ >>  __start__) & CANAL_BIT_MASK(__length__))

// CANAL_SET_BITS will take the first (__length__) bits from (__input__) and set them
// to (__output__) from bit position (__start__) to (__start__ + __length__)
#define CANAL_SET_BITS(__input__, __output__, __start__, __length__) 	(CANAL_CLEAR_BITS(__output__, __start__, __length__) | ((uint64_t)(__input__) & CANAL_BIT_MASK(__length__)) << __start__)

// CANAL_TWOS_COMPLEMENT will take the twos complement of (__data__)
#define CANAL_TWOS_COMPLEMENT(__data__, __length__) (((~(__data__))+1) & CANAL_BIT_MASK(__length__))

// CANAL_IS_NEGATIVE checks the msb to see if (__data__) is negative
#define CANAL_IS_NEGATIVE(__data__, __length__) (((__data__) & ((uint64_t)1 << (__length__-1))) != 0)

// CANAL_TO_SIGNED converts (__data__) to a signed value
#define CANAL_TO_SIGNED(__data__, __length__)   ((signed)(CANAL_IS_NEGATIVE(__data__, __length__) ? -(CANAL_TWOS_COMPLEMENT(__data__, __length__)) : __data__))

    
/*********************************************************
*                 GLOBAL STRUCT DEFINITIONS
*********************************************************/

volatile TsAMK1_SetPoints1        AMK1_SetPoints1;
volatile TsAMK1_ActualValues2     AMK1_ActualValues2;
volatile TsAMK1_ActualValues1     AMK1_ActualValues1;
volatile TsAMK0_ActualValues1     AMK0_ActualValues1;
volatile TsAMK0_ActualValues2     AMK0_ActualValues2;
volatile TsAMK0_SetPoints1        AMK0_SetPoints1;
volatile TsContactor_States       Contactor_States;
volatile TsPack_State             Pack_State;
volatile TsPack_Current_Limits    Pack_Current_Limits;
volatile TsPack_SOC               Pack_SOC;
volatile TsContactor_Feedback     Contactor_Feedback;

/*********************************************************
 *                    HELPER FUNCTIONS
 *********************************************************/

static uint64_t getDataWordFromByteArray(uint8_t* rawData) {
	uint64_t ret = 0;

	for (int8_t i = 7; i >= 0; i--) {
		ret <<= 8;
		ret += rawData[i];
	}

	return ret;
}

static void setDataWordIntoByteArray(uint64_t dataWord, uint8_t *dataOutput) {
	for (int8_t i = 0; i < 8; i++) {
		dataOutput[i] = (uint8_t) (dataWord & CANAL_BIT_MASK(8));
		dataWord >>= 8;
	}
}

static uint64_t shift_endianness(uint64_t val, uint8_t num_bits) {
	unsigned long long result = 0;
	unsigned long long temp_value = val;
	unsigned int num_bytes = (num_bits + 7) / 8;
	unsigned int num_shifts = (num_bytes - 1) * 8;
	
	for (unsigned int i = 0; i < num_bytes; ++i) {
		result |= ((temp_value >> (num_shifts - (i * 8))) & 0xFFULL) << (i * 8);
	}

	return result;
}

/*********************************************************
 *                    CAN FUNCTIONS
 *********************************************************/
 
TeCanALRet Unmarshal_AMK1_ActualValues2(uint8_t *RxData) {
	uint64_t data;
	TsAMK1_ActualValues2 temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.AMK_TempMotor = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES2_AMK_TEMPMOTOR_START,
            AMK1_ACTUALVALUES2_AMK_TEMPMOTOR_LENGTH),
            AMK1_ACTUALVALUES2_AMK_TEMPMOTOR_LENGTH);
            
        temp.AMK_TempInverter = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES2_AMK_TEMPINVERTER_START,
            AMK1_ACTUALVALUES2_AMK_TEMPINVERTER_LENGTH),
            AMK1_ACTUALVALUES2_AMK_TEMPINVERTER_LENGTH);
            
        temp.AMK_ErrorInfo = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES2_AMK_ERRORINFO_START,
            AMK1_ACTUALVALUES2_AMK_ERRORINFO_LENGTH);
            
        temp.AMK_TempIGBT = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES2_AMK_TEMPIGBT_START,
            AMK1_ACTUALVALUES2_AMK_TEMPIGBT_LENGTH),
            AMK1_ACTUALVALUES2_AMK_TEMPIGBT_LENGTH);
            
	//  Apply linear conversion
	temp.AMK_TempMotor = (temp.AMK_TempMotor
			* AMK1_ACTUALVALUES2_AMK_TEMPMOTOR_FACTOR)
			+ AMK1_ACTUALVALUES2_AMK_TEMPMOTOR_OFFSET; 
            
	temp.AMK_TempInverter = (temp.AMK_TempInverter
			* AMK1_ACTUALVALUES2_AMK_TEMPINVERTER_FACTOR)
			+ AMK1_ACTUALVALUES2_AMK_TEMPINVERTER_OFFSET; 
            
	temp.AMK_ErrorInfo = (temp.AMK_ErrorInfo
			* AMK1_ACTUALVALUES2_AMK_ERRORINFO_FACTOR)
			+ AMK1_ACTUALVALUES2_AMK_ERRORINFO_OFFSET; 
            
	temp.AMK_TempIGBT = (temp.AMK_TempIGBT
			* AMK1_ACTUALVALUES2_AMK_TEMPIGBT_FACTOR)
			+ AMK1_ACTUALVALUES2_AMK_TEMPIGBT_OFFSET; 
            
	//  Writing to global struct instance
	AMK1_ActualValues2 = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_AMK1_ActualValues1(uint8_t *RxData) {
	uint64_t data;
	TsAMK1_ActualValues1 temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.AMK_bSystemReady = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BSYSTEMREADY_START,
            AMK1_ACTUALVALUES1_AMK_BSYSTEMREADY_LENGTH);
            
        temp.AMK_bError = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BERROR_START,
            AMK1_ACTUALVALUES1_AMK_BERROR_LENGTH);
            
        temp.AMK_bWarn = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BWARN_START,
            AMK1_ACTUALVALUES1_AMK_BWARN_LENGTH);
            
        temp.AMK_bQuitDcOn = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BQUITDCON_START,
            AMK1_ACTUALVALUES1_AMK_BQUITDCON_LENGTH);
            
        temp.AMK_bDcOn = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BDCON_START,
            AMK1_ACTUALVALUES1_AMK_BDCON_LENGTH);
            
        temp.AMK_bQuitInverterOn = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BQUITINVERTERON_START,
            AMK1_ACTUALVALUES1_AMK_BQUITINVERTERON_LENGTH);
            
        temp.AMK_bInverterOn = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BINVERTERON_START,
            AMK1_ACTUALVALUES1_AMK_BINVERTERON_LENGTH);
            
        temp.AMK_bDerating = CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_BDERATING_START,
            AMK1_ACTUALVALUES1_AMK_BDERATING_LENGTH);
            
        temp.AMK_ActualVelocity = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_ACTUALVELOCITY_START,
            AMK1_ACTUALVALUES1_AMK_ACTUALVELOCITY_LENGTH),
            AMK1_ACTUALVALUES1_AMK_ACTUALVELOCITY_LENGTH);
            
        temp.AMK_TorqueCurrent = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_TORQUECURRENT_START,
            AMK1_ACTUALVALUES1_AMK_TORQUECURRENT_LENGTH),
            AMK1_ACTUALVALUES1_AMK_TORQUECURRENT_LENGTH);
            
        temp.AMK_MagnetizingCurrent = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK1_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_START,
            AMK1_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_LENGTH),
            AMK1_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_LENGTH);
            
	//  Apply linear conversion
	temp.AMK_bSystemReady = (temp.AMK_bSystemReady
			* AMK1_ACTUALVALUES1_AMK_BSYSTEMREADY_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BSYSTEMREADY_OFFSET; 
            
	temp.AMK_bError = (temp.AMK_bError
			* AMK1_ACTUALVALUES1_AMK_BERROR_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BERROR_OFFSET; 
            
	temp.AMK_bWarn = (temp.AMK_bWarn
			* AMK1_ACTUALVALUES1_AMK_BWARN_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BWARN_OFFSET; 
            
	temp.AMK_bQuitDcOn = (temp.AMK_bQuitDcOn
			* AMK1_ACTUALVALUES1_AMK_BQUITDCON_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BQUITDCON_OFFSET; 
            
	temp.AMK_bDcOn = (temp.AMK_bDcOn
			* AMK1_ACTUALVALUES1_AMK_BDCON_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BDCON_OFFSET; 
            
	temp.AMK_bQuitInverterOn = (temp.AMK_bQuitInverterOn
			* AMK1_ACTUALVALUES1_AMK_BQUITINVERTERON_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BQUITINVERTERON_OFFSET; 
            
	temp.AMK_bInverterOn = (temp.AMK_bInverterOn
			* AMK1_ACTUALVALUES1_AMK_BINVERTERON_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BINVERTERON_OFFSET; 
            
	temp.AMK_bDerating = (temp.AMK_bDerating
			* AMK1_ACTUALVALUES1_AMK_BDERATING_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_BDERATING_OFFSET; 
            
	temp.AMK_ActualVelocity = (temp.AMK_ActualVelocity
			* AMK1_ACTUALVALUES1_AMK_ACTUALVELOCITY_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_ACTUALVELOCITY_OFFSET; 
            
	temp.AMK_TorqueCurrent = (temp.AMK_TorqueCurrent
			* AMK1_ACTUALVALUES1_AMK_TORQUECURRENT_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_TORQUECURRENT_OFFSET; 
            
	temp.AMK_MagnetizingCurrent = (temp.AMK_MagnetizingCurrent
			* AMK1_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_FACTOR)
			+ AMK1_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_OFFSET; 
            
	//  Writing to global struct instance
	AMK1_ActualValues1 = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_AMK0_ActualValues1(uint8_t *RxData) {
	uint64_t data;
	TsAMK0_ActualValues1 temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.AMK_bSystemReady = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BSYSTEMREADY_START,
            AMK0_ACTUALVALUES1_AMK_BSYSTEMREADY_LENGTH);
            
        temp.AMK_bError = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BERROR_START,
            AMK0_ACTUALVALUES1_AMK_BERROR_LENGTH);
            
        temp.AMK_bWarn = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BWARN_START,
            AMK0_ACTUALVALUES1_AMK_BWARN_LENGTH);
            
        temp.AMK_bQuitDcOn = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BQUITDCON_START,
            AMK0_ACTUALVALUES1_AMK_BQUITDCON_LENGTH);
            
        temp.AMK_bDcOn = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BDCON_START,
            AMK0_ACTUALVALUES1_AMK_BDCON_LENGTH);
            
        temp.AMK_bQuitInverterOn = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BQUITINVERTERON_START,
            AMK0_ACTUALVALUES1_AMK_BQUITINVERTERON_LENGTH);
            
        temp.AMK_bInverterOn = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BINVERTERON_START,
            AMK0_ACTUALVALUES1_AMK_BINVERTERON_LENGTH);
            
        temp.AMK_bDerating = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_BDERATING_START,
            AMK0_ACTUALVALUES1_AMK_BDERATING_LENGTH);
            
        temp.AMK_ActualVelocity = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_ACTUALVELOCITY_START,
            AMK0_ACTUALVALUES1_AMK_ACTUALVELOCITY_LENGTH),
            AMK0_ACTUALVALUES1_AMK_ACTUALVELOCITY_LENGTH);
            
        temp.AMK_TorqueCurrent = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_TORQUECURRENT_START,
            AMK0_ACTUALVALUES1_AMK_TORQUECURRENT_LENGTH),
            AMK0_ACTUALVALUES1_AMK_TORQUECURRENT_LENGTH);
            
        temp.AMK_MagnetizingCurrent = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_START,
            AMK0_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_LENGTH),
            AMK0_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_LENGTH);
            
	//  Apply linear conversion
	temp.AMK_bSystemReady = (temp.AMK_bSystemReady
			* AMK0_ACTUALVALUES1_AMK_BSYSTEMREADY_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BSYSTEMREADY_OFFSET; 
            
	temp.AMK_bError = (temp.AMK_bError
			* AMK0_ACTUALVALUES1_AMK_BERROR_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BERROR_OFFSET; 
            
	temp.AMK_bWarn = (temp.AMK_bWarn
			* AMK0_ACTUALVALUES1_AMK_BWARN_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BWARN_OFFSET; 
            
	temp.AMK_bQuitDcOn = (temp.AMK_bQuitDcOn
			* AMK0_ACTUALVALUES1_AMK_BQUITDCON_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BQUITDCON_OFFSET; 
            
	temp.AMK_bDcOn = (temp.AMK_bDcOn
			* AMK0_ACTUALVALUES1_AMK_BDCON_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BDCON_OFFSET; 
            
	temp.AMK_bQuitInverterOn = (temp.AMK_bQuitInverterOn
			* AMK0_ACTUALVALUES1_AMK_BQUITINVERTERON_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BQUITINVERTERON_OFFSET; 
            
	temp.AMK_bInverterOn = (temp.AMK_bInverterOn
			* AMK0_ACTUALVALUES1_AMK_BINVERTERON_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BINVERTERON_OFFSET; 
            
	temp.AMK_bDerating = (temp.AMK_bDerating
			* AMK0_ACTUALVALUES1_AMK_BDERATING_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_BDERATING_OFFSET; 
            
	temp.AMK_ActualVelocity = (temp.AMK_ActualVelocity
			* AMK0_ACTUALVALUES1_AMK_ACTUALVELOCITY_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_ACTUALVELOCITY_OFFSET; 
            
	temp.AMK_TorqueCurrent = (temp.AMK_TorqueCurrent
			* AMK0_ACTUALVALUES1_AMK_TORQUECURRENT_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_TORQUECURRENT_OFFSET; 
            
	temp.AMK_MagnetizingCurrent = (temp.AMK_MagnetizingCurrent
			* AMK0_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_FACTOR)
			+ AMK0_ACTUALVALUES1_AMK_MAGNETIZINGCURRENT_OFFSET; 
            
	//  Writing to global struct instance
	AMK0_ActualValues1 = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_AMK0_ActualValues2(uint8_t *RxData) {
	uint64_t data;
	TsAMK0_ActualValues2 temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.AMK_TempMotor = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES2_AMK_TEMPMOTOR_START,
            AMK0_ACTUALVALUES2_AMK_TEMPMOTOR_LENGTH),
            AMK0_ACTUALVALUES2_AMK_TEMPMOTOR_LENGTH);
            
        temp.AMK_TempInverter = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES2_AMK_TEMPINVERTER_START,
            AMK0_ACTUALVALUES2_AMK_TEMPINVERTER_LENGTH),
            AMK0_ACTUALVALUES2_AMK_TEMPINVERTER_LENGTH);
            
        temp.AMK_ErrorInfo = CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES2_AMK_ERRORINFO_START,
            AMK0_ACTUALVALUES2_AMK_ERRORINFO_LENGTH);
            
        temp.AMK_TempIGBT = CANAL_TO_SIGNED(CANAL_GET_BITS(data,
            AMK0_ACTUALVALUES2_AMK_TEMPIGBT_START,
            AMK0_ACTUALVALUES2_AMK_TEMPIGBT_LENGTH),
            AMK0_ACTUALVALUES2_AMK_TEMPIGBT_LENGTH);
            
	//  Apply linear conversion
	temp.AMK_TempMotor = (temp.AMK_TempMotor
			* AMK0_ACTUALVALUES2_AMK_TEMPMOTOR_FACTOR)
			+ AMK0_ACTUALVALUES2_AMK_TEMPMOTOR_OFFSET; 
            
	temp.AMK_TempInverter = (temp.AMK_TempInverter
			* AMK0_ACTUALVALUES2_AMK_TEMPINVERTER_FACTOR)
			+ AMK0_ACTUALVALUES2_AMK_TEMPINVERTER_OFFSET; 
            
	temp.AMK_ErrorInfo = (temp.AMK_ErrorInfo
			* AMK0_ACTUALVALUES2_AMK_ERRORINFO_FACTOR)
			+ AMK0_ACTUALVALUES2_AMK_ERRORINFO_OFFSET; 
            
	temp.AMK_TempIGBT = (temp.AMK_TempIGBT
			* AMK0_ACTUALVALUES2_AMK_TEMPIGBT_FACTOR)
			+ AMK0_ACTUALVALUES2_AMK_TEMPIGBT_OFFSET; 
            
	//  Writing to global struct instance
	AMK0_ActualValues2 = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_Pack_State(uint8_t *RxData) {
	uint64_t data;
	TsPack_State temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.Pack_Current = CANAL_GET_BITS(data,
            PACK_STATE_PACK_CURRENT_START,
            PACK_STATE_PACK_CURRENT_LENGTH);
            
        temp.Pack_Inst_Voltage = CANAL_GET_BITS(data,
            PACK_STATE_PACK_INST_VOLTAGE_START,
            PACK_STATE_PACK_INST_VOLTAGE_LENGTH);
            
        temp.Avg_Cell_Voltage = CANAL_GET_BITS(data,
            PACK_STATE_AVG_CELL_VOLTAGE_START,
            PACK_STATE_AVG_CELL_VOLTAGE_LENGTH);
            
        temp.Populated_Cells = CANAL_GET_BITS(data,
            PACK_STATE_POPULATED_CELLS_START,
            PACK_STATE_POPULATED_CELLS_LENGTH);
            
	//  Apply linear conversion
	temp.Pack_Current = (temp.Pack_Current
			* PACK_STATE_PACK_CURRENT_FACTOR)
			+ PACK_STATE_PACK_CURRENT_OFFSET; 
            
	temp.Pack_Inst_Voltage = (temp.Pack_Inst_Voltage
			* PACK_STATE_PACK_INST_VOLTAGE_FACTOR)
			+ PACK_STATE_PACK_INST_VOLTAGE_OFFSET; 
            
	temp.Avg_Cell_Voltage = (temp.Avg_Cell_Voltage
			* PACK_STATE_AVG_CELL_VOLTAGE_FACTOR)
			+ PACK_STATE_AVG_CELL_VOLTAGE_OFFSET; 
            
	temp.Populated_Cells = (temp.Populated_Cells
			* PACK_STATE_POPULATED_CELLS_FACTOR)
			+ PACK_STATE_POPULATED_CELLS_OFFSET; 
            
	//  Writing to global struct instance
	Pack_State = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_Pack_Current_Limits(uint8_t *RxData) {
	uint64_t data;
	TsPack_Current_Limits temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.Pack_CCL = CANAL_GET_BITS(data,
            PACK_CURRENT_LIMITS_PACK_CCL_START,
            PACK_CURRENT_LIMITS_PACK_CCL_LENGTH);
            
        temp.Pack_DCL = CANAL_GET_BITS(data,
            PACK_CURRENT_LIMITS_PACK_DCL_START,
            PACK_CURRENT_LIMITS_PACK_DCL_LENGTH);
            
	//  Apply linear conversion
	temp.Pack_CCL = (temp.Pack_CCL
			* PACK_CURRENT_LIMITS_PACK_CCL_FACTOR)
			+ PACK_CURRENT_LIMITS_PACK_CCL_OFFSET; 
            
	temp.Pack_DCL = (temp.Pack_DCL
			* PACK_CURRENT_LIMITS_PACK_DCL_FACTOR)
			+ PACK_CURRENT_LIMITS_PACK_DCL_OFFSET; 
            
	//  Writing to global struct instance
	Pack_Current_Limits = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_Pack_SOC(uint8_t *RxData) {
	uint64_t data;
	TsPack_SOC temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.Pack_SOC = CANAL_GET_BITS(data,
            PACK_SOC_PACK_SOC_START,
            PACK_SOC_PACK_SOC_LENGTH);
            
        temp.Maximum_Pack_Voltage = CANAL_GET_BITS(data,
            PACK_SOC_MAXIMUM_PACK_VOLTAGE_START,
            PACK_SOC_MAXIMUM_PACK_VOLTAGE_LENGTH);
            
	//  Apply linear conversion
	temp.Pack_SOC = (temp.Pack_SOC
			* PACK_SOC_PACK_SOC_FACTOR)
			+ PACK_SOC_PACK_SOC_OFFSET; 
            
	temp.Maximum_Pack_Voltage = (temp.Maximum_Pack_Voltage
			* PACK_SOC_MAXIMUM_PACK_VOLTAGE_FACTOR)
			+ PACK_SOC_MAXIMUM_PACK_VOLTAGE_OFFSET; 
            
	//  Writing to global struct instance
	Pack_SOC = temp;

	return CANAL_OK;
}
TeCanALRet Unmarshal_Contactor_Feedback(uint8_t *RxData) {
	uint64_t data;
	TsContactor_Feedback temp;

	data = getDataWordFromByteArray(RxData);
    
	//  Read raw bits
        temp.Pack_Precharge_Feedback = CANAL_GET_BITS(data,
            CONTACTOR_FEEDBACK_PACK_PRECHARGE_FEEDBACK_START,
            CONTACTOR_FEEDBACK_PACK_PRECHARGE_FEEDBACK_LENGTH);
            
        temp.Pack_Negative_Feedback = CANAL_GET_BITS(data,
            CONTACTOR_FEEDBACK_PACK_NEGATIVE_FEEDBACK_START,
            CONTACTOR_FEEDBACK_PACK_NEGATIVE_FEEDBACK_LENGTH);
            
        temp.Pack_Positive_Feedback = CANAL_GET_BITS(data,
            CONTACTOR_FEEDBACK_PACK_POSITIVE_FEEDBACK_START,
            CONTACTOR_FEEDBACK_PACK_POSITIVE_FEEDBACK_LENGTH);
            
	//  Apply linear conversion
	temp.Pack_Precharge_Feedback = (temp.Pack_Precharge_Feedback
			* CONTACTOR_FEEDBACK_PACK_PRECHARGE_FEEDBACK_FACTOR)
			+ CONTACTOR_FEEDBACK_PACK_PRECHARGE_FEEDBACK_OFFSET; 
            
	temp.Pack_Negative_Feedback = (temp.Pack_Negative_Feedback
			* CONTACTOR_FEEDBACK_PACK_NEGATIVE_FEEDBACK_FACTOR)
			+ CONTACTOR_FEEDBACK_PACK_NEGATIVE_FEEDBACK_OFFSET; 
            
	temp.Pack_Positive_Feedback = (temp.Pack_Positive_Feedback
			* CONTACTOR_FEEDBACK_PACK_POSITIVE_FEEDBACK_FACTOR)
			+ CONTACTOR_FEEDBACK_PACK_POSITIVE_FEEDBACK_OFFSET; 
            
	//  Writing to global struct instance
	Contactor_Feedback = temp;

	return CANAL_OK;
}

TeCanALRet Marshal_AMK1_SetPoints1(uint8_t *TxData) {
	uint64_t dataWord = 0;
	TsAMK1_SetPoints1 temp;
	TeCanALRet ret;

	temp = AMK1_SetPoints1;
    
    // Reverse linear conversion
	temp.AMK_bInverterOn = (temp.AMK_bInverterOn
			- AMK1_SETPOINTS1_AMK_BINVERTERON_OFFSET)
			/ AMK1_SETPOINTS1_AMK_BINVERTERON_FACTOR;
            
	temp.AMK_bDcOn = (temp.AMK_bDcOn
			- AMK1_SETPOINTS1_AMK_BDCON_OFFSET)
			/ AMK1_SETPOINTS1_AMK_BDCON_FACTOR;
            
	temp.AMK_bEnable = (temp.AMK_bEnable
			- AMK1_SETPOINTS1_AMK_BENABLE_OFFSET)
			/ AMK1_SETPOINTS1_AMK_BENABLE_FACTOR;
            
	temp.AMK_bErrorReset = (temp.AMK_bErrorReset
			- AMK1_SETPOINTS1_AMK_BERRORRESET_OFFSET)
			/ AMK1_SETPOINTS1_AMK_BERRORRESET_FACTOR;
            
	temp.AMK_TargetVelocity = (temp.AMK_TargetVelocity
			- AMK1_SETPOINTS1_AMK_TARGETVELOCITY_OFFSET)
			/ AMK1_SETPOINTS1_AMK_TARGETVELOCITY_FACTOR;
            
	temp.AMK_TorqueLimitPositiv = (temp.AMK_TorqueLimitPositiv
			- AMK1_SETPOINTS1_AMK_TORQUELIMITPOSITIV_OFFSET)
			/ AMK1_SETPOINTS1_AMK_TORQUELIMITPOSITIV_FACTOR;
            
	temp.AMK_TorqueLimitNegativ = (temp.AMK_TorqueLimitNegativ
			- AMK1_SETPOINTS1_AMK_TORQUELIMITNEGATIV_OFFSET)
			/ AMK1_SETPOINTS1_AMK_TORQUELIMITNEGATIV_FACTOR;
            
    // CANAL_SET_BITS only writes to correct bits, dataWord is not being overwritten
	dataWord = CANAL_SET_BITS(temp.AMK_bInverterOn, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_BINVERTERON_START,
			AMK1_SETPOINTS1_AMK_BINVERTERON_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_bDcOn, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_BDCON_START,
			AMK1_SETPOINTS1_AMK_BDCON_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_bEnable, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_BENABLE_START,
			AMK1_SETPOINTS1_AMK_BENABLE_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_bErrorReset, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_BERRORRESET_START,
			AMK1_SETPOINTS1_AMK_BERRORRESET_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_TargetVelocity, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_TARGETVELOCITY_START,
			AMK1_SETPOINTS1_AMK_TARGETVELOCITY_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_TorqueLimitPositiv, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_TORQUELIMITPOSITIV_START,
			AMK1_SETPOINTS1_AMK_TORQUELIMITPOSITIV_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_TorqueLimitNegativ, dataWord,
            //little_endian
            AMK1_SETPOINTS1_AMK_TORQUELIMITNEGATIV_START,
			AMK1_SETPOINTS1_AMK_TORQUELIMITNEGATIV_LENGTH);
            
	setDataWordIntoByteArray(dataWord, TxData);

	return CANAL_OK;
}
TeCanALRet Marshal_AMK0_SetPoints1(uint8_t *TxData) {
	uint64_t dataWord = 0;
	TsAMK0_SetPoints1 temp;
	TeCanALRet ret;

	temp = AMK0_SetPoints1;
    
    // Reverse linear conversion
	temp.AMK_bInverterOn = (temp.AMK_bInverterOn
			- AMK0_SETPOINTS1_AMK_BINVERTERON_OFFSET)
			/ AMK0_SETPOINTS1_AMK_BINVERTERON_FACTOR;
            
	temp.AMK_bDcOn = (temp.AMK_bDcOn
			- AMK0_SETPOINTS1_AMK_BDCON_OFFSET)
			/ AMK0_SETPOINTS1_AMK_BDCON_FACTOR;
            
	temp.AMK_bEnable = (temp.AMK_bEnable
			- AMK0_SETPOINTS1_AMK_BENABLE_OFFSET)
			/ AMK0_SETPOINTS1_AMK_BENABLE_FACTOR;
            
	temp.AMK_bErrorReset = (temp.AMK_bErrorReset
			- AMK0_SETPOINTS1_AMK_BERRORRESET_OFFSET)
			/ AMK0_SETPOINTS1_AMK_BERRORRESET_FACTOR;
            
	temp.AMK_TargetVelocity = (temp.AMK_TargetVelocity
			- AMK0_SETPOINTS1_AMK_TARGETVELOCITY_OFFSET)
			/ AMK0_SETPOINTS1_AMK_TARGETVELOCITY_FACTOR;
            
	temp.AMK_TorqueLimitPositiv = (temp.AMK_TorqueLimitPositiv
			- AMK0_SETPOINTS1_AMK_TORQUELIMITPOSITIV_OFFSET)
			/ AMK0_SETPOINTS1_AMK_TORQUELIMITPOSITIV_FACTOR;
            
	temp.AMK_TorqueLimitNegativ = (temp.AMK_TorqueLimitNegativ
			- AMK0_SETPOINTS1_AMK_TORQUELIMITNEGATIV_OFFSET)
			/ AMK0_SETPOINTS1_AMK_TORQUELIMITNEGATIV_FACTOR;
            
    // CANAL_SET_BITS only writes to correct bits, dataWord is not being overwritten
	dataWord = CANAL_SET_BITS(temp.AMK_bInverterOn, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_BINVERTERON_START,
			AMK0_SETPOINTS1_AMK_BINVERTERON_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_bDcOn, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_BDCON_START,
			AMK0_SETPOINTS1_AMK_BDCON_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_bEnable, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_BENABLE_START,
			AMK0_SETPOINTS1_AMK_BENABLE_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_bErrorReset, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_BERRORRESET_START,
			AMK0_SETPOINTS1_AMK_BERRORRESET_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_TargetVelocity, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_TARGETVELOCITY_START,
			AMK0_SETPOINTS1_AMK_TARGETVELOCITY_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_TorqueLimitPositiv, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_TORQUELIMITPOSITIV_START,
			AMK0_SETPOINTS1_AMK_TORQUELIMITPOSITIV_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.AMK_TorqueLimitNegativ, dataWord,
            //little_endian
            AMK0_SETPOINTS1_AMK_TORQUELIMITNEGATIV_START,
			AMK0_SETPOINTS1_AMK_TORQUELIMITNEGATIV_LENGTH);
            
	setDataWordIntoByteArray(dataWord, TxData);

	return CANAL_OK;
}
TeCanALRet Marshal_Contactor_States(uint8_t *TxData) {
	uint64_t dataWord = 0;
	TsContactor_States temp;
	TeCanALRet ret;

	temp = Contactor_States;
    
    // Reverse linear conversion
	temp.Pack_Positive = (temp.Pack_Positive
			- CONTACTOR_STATES_PACK_POSITIVE_OFFSET)
			/ CONTACTOR_STATES_PACK_POSITIVE_FACTOR;
            
	temp.Pack_Precharge = (temp.Pack_Precharge
			- CONTACTOR_STATES_PACK_PRECHARGE_OFFSET)
			/ CONTACTOR_STATES_PACK_PRECHARGE_FACTOR;
            
	temp.Pack_Negative = (temp.Pack_Negative
			- CONTACTOR_STATES_PACK_NEGATIVE_OFFSET)
			/ CONTACTOR_STATES_PACK_NEGATIVE_FACTOR;
            
    // CANAL_SET_BITS only writes to correct bits, dataWord is not being overwritten
	dataWord = CANAL_SET_BITS(temp.Pack_Positive, dataWord,
            //little_endian
            CONTACTOR_STATES_PACK_POSITIVE_START,
			CONTACTOR_STATES_PACK_POSITIVE_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.Pack_Precharge, dataWord,
            //little_endian
            CONTACTOR_STATES_PACK_PRECHARGE_START,
			CONTACTOR_STATES_PACK_PRECHARGE_LENGTH);
            
	dataWord = CANAL_SET_BITS(temp.Pack_Negative, dataWord,
            //little_endian
            CONTACTOR_STATES_PACK_NEGATIVE_START,
			CONTACTOR_STATES_PACK_NEGATIVE_LENGTH);
            
	setDataWordIntoByteArray(dataWord, TxData);

	return CANAL_OK;
}

/*********************************************************
 *               FUNCTION POINTER TABLE
 *********************************************************/

static const struct {
	TeMessageID ID;
	BinaryUnmarshaller *Unmarshal;
} CANAL_RX_MESSAGE_TABLE[NUM_RX_MESSAGES] = {
	{ AMK1_ACTUALVALUES2_CANAL_ID,     &Unmarshal_AMK1_ActualValues2 },
	{ AMK1_ACTUALVALUES1_CANAL_ID,     &Unmarshal_AMK1_ActualValues1 },
	{ AMK0_ACTUALVALUES1_CANAL_ID,     &Unmarshal_AMK0_ActualValues1 },
	{ AMK0_ACTUALVALUES2_CANAL_ID,     &Unmarshal_AMK0_ActualValues2 },
	{ PACK_STATE_CANAL_ID,             &Unmarshal_Pack_State },
	{ PACK_CURRENT_LIMITS_CANAL_ID,    &Unmarshal_Pack_Current_Limits },
	{ PACK_SOC_CANAL_ID,               &Unmarshal_Pack_SOC },
	{ CONTACTOR_FEEDBACK_CANAL_ID,     &Unmarshal_Contactor_Feedback },
};

static const struct {
	TeMessageID ID;
	BinaryMarshaller *Marshal;
    uint32_t dlc;
} CANAL_TX_MESSAGE_TABLE[NUM_TX_MESSAGES] = {
	{ AMK1_SETPOINTS1_CANAL_ID,        &Marshal_AMK1_SetPoints1,        AMK1_SETPOINTS1_DATA_LENGTH },
	{ AMK0_SETPOINTS1_CANAL_ID,        &Marshal_AMK0_SetPoints1,        AMK0_SETPOINTS1_DATA_LENGTH },
	{ CONTACTOR_STATES_CANAL_ID,       &Marshal_Contactor_States,       CONTACTOR_STATES_DATA_LENGTH },
};

static const struct {
	TeMessageID ID;
	CanALPrinter* printer;
} CANAL_PRINTER_TABLE[TOTAL_MESSAGES] = {
	{ AMK1_SETPOINTS1_CANAL_ID,        &Print_AMK1_SetPoints1 },
	{ AMK1_ACTUALVALUES2_CANAL_ID,     &Print_AMK1_ActualValues2 },
	{ AMK1_ACTUALVALUES1_CANAL_ID,     &Print_AMK1_ActualValues1 },
	{ AMK0_ACTUALVALUES1_CANAL_ID,     &Print_AMK0_ActualValues1 },
	{ AMK0_ACTUALVALUES2_CANAL_ID,     &Print_AMK0_ActualValues2 },
	{ AMK0_SETPOINTS1_CANAL_ID,        &Print_AMK0_SetPoints1 },
	{ CONTACTOR_STATES_CANAL_ID,       &Print_Contactor_States },
	{ PACK_STATE_CANAL_ID,             &Print_Pack_State },
	{ PACK_CURRENT_LIMITS_CANAL_ID,    &Print_Pack_Current_Limits },
	{ PACK_SOC_CANAL_ID,               &Print_Pack_SOC },
	{ CONTACTOR_FEEDBACK_CANAL_ID,     &Print_Contactor_Feedback },
};

/*********************************************************
 *           FUNCTION POINTER TABLE GETTERS
 *********************************************************/

 static TeCanALRet getBinaryUnmarshaller(uint32_t *ID, BinaryUnmarshaller **pUnmarshal) {
	for (int i = 0; i < NUM_RX_MESSAGES; i++) {
		if ((*ID) == CANAL_RX_MESSAGE_TABLE[i].ID) {
			*pUnmarshal = CANAL_RX_MESSAGE_TABLE[i].Unmarshal;

			return CANAL_OK;
		}
	}

	return CANAL_UNSUPPORTED_RX_MESSAGE;
}

static TeCanALRet getBinaryMarshaller(TeMessageID *ID, BinaryMarshaller **pMarshal) {
	for (int i = 0; i < NUM_TX_MESSAGES; i++) {
		if ((*ID) == CANAL_TX_MESSAGE_TABLE[i].ID) {
			*pMarshal = CANAL_TX_MESSAGE_TABLE[i].Marshal;

			return CANAL_OK;
		}
	}

	return CANAL_UNSUPPORTED_TX_MESSAGE;
}

/*********************************************************
 *             PUBLIC FUNCTION DEFINITIONS
 *********************************************************/
 
void Print_AMK1_SetPoints1() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: AMK1_SetPoints1");
    CANAL_PRINT("%sAMK_bInverterOn: %u", sep, AMK1_SetPoints1.AMK_bInverterOn);
    CANAL_PRINT("%sAMK_bDcOn: %u", sep, AMK1_SetPoints1.AMK_bDcOn);
    CANAL_PRINT("%sAMK_bEnable: %u", sep, AMK1_SetPoints1.AMK_bEnable);
    CANAL_PRINT("%sAMK_bErrorReset: %u", sep, AMK1_SetPoints1.AMK_bErrorReset);
    CANAL_PRINT("%sAMK_TargetVelocity: %d", sep, AMK1_SetPoints1.AMK_TargetVelocity);
    CANAL_PRINT("%sAMK_TorqueLimitPositiv: %d", sep, AMK1_SetPoints1.AMK_TorqueLimitPositiv);
    CANAL_PRINT("%sAMK_TorqueLimitNegativ: %d", sep, AMK1_SetPoints1.AMK_TorqueLimitNegativ);
    CANAL_PRINT("\n\r");
}
void Print_AMK1_ActualValues2() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: AMK1_ActualValues2");
    CANAL_PRINT("%sAMK_TempMotor: %f", sep, AMK1_ActualValues2.AMK_TempMotor);
    CANAL_PRINT("%sAMK_TempInverter: %f", sep, AMK1_ActualValues2.AMK_TempInverter);
    CANAL_PRINT("%sAMK_ErrorInfo: %u", sep, AMK1_ActualValues2.AMK_ErrorInfo);
    CANAL_PRINT("%sAMK_TempIGBT: %f", sep, AMK1_ActualValues2.AMK_TempIGBT);
    CANAL_PRINT("\n\r");
}
void Print_AMK1_ActualValues1() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: AMK1_ActualValues1");
    CANAL_PRINT("%sAMK_bSystemReady: %u", sep, AMK1_ActualValues1.AMK_bSystemReady);
    CANAL_PRINT("%sAMK_bError: %u", sep, AMK1_ActualValues1.AMK_bError);
    CANAL_PRINT("%sAMK_bWarn: %u", sep, AMK1_ActualValues1.AMK_bWarn);
    CANAL_PRINT("%sAMK_bQuitDcOn: %u", sep, AMK1_ActualValues1.AMK_bQuitDcOn);
    CANAL_PRINT("%sAMK_bDcOn: %u", sep, AMK1_ActualValues1.AMK_bDcOn);
    CANAL_PRINT("%sAMK_bQuitInverterOn: %u", sep, AMK1_ActualValues1.AMK_bQuitInverterOn);
    CANAL_PRINT("%sAMK_bInverterOn: %u", sep, AMK1_ActualValues1.AMK_bInverterOn);
    CANAL_PRINT("%sAMK_bDerating: %u", sep, AMK1_ActualValues1.AMK_bDerating);
    CANAL_PRINT("%sAMK_ActualVelocity: %d", sep, AMK1_ActualValues1.AMK_ActualVelocity);
    CANAL_PRINT("%sAMK_TorqueCurrent: %d", sep, AMK1_ActualValues1.AMK_TorqueCurrent);
    CANAL_PRINT("%sAMK_MagnetizingCurrent: %d", sep, AMK1_ActualValues1.AMK_MagnetizingCurrent);
    CANAL_PRINT("\n\r");
}
void Print_AMK0_ActualValues1() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: AMK0_ActualValues1");
    CANAL_PRINT("%sAMK_bSystemReady: %u", sep, AMK0_ActualValues1.AMK_bSystemReady);
    CANAL_PRINT("%sAMK_bError: %u", sep, AMK0_ActualValues1.AMK_bError);
    CANAL_PRINT("%sAMK_bWarn: %u", sep, AMK0_ActualValues1.AMK_bWarn);
    CANAL_PRINT("%sAMK_bQuitDcOn: %u", sep, AMK0_ActualValues1.AMK_bQuitDcOn);
    CANAL_PRINT("%sAMK_bDcOn: %u", sep, AMK0_ActualValues1.AMK_bDcOn);
    CANAL_PRINT("%sAMK_bQuitInverterOn: %u", sep, AMK0_ActualValues1.AMK_bQuitInverterOn);
    CANAL_PRINT("%sAMK_bInverterOn: %u", sep, AMK0_ActualValues1.AMK_bInverterOn);
    CANAL_PRINT("%sAMK_bDerating: %u", sep, AMK0_ActualValues1.AMK_bDerating);
    CANAL_PRINT("%sAMK_ActualVelocity: %d", sep, AMK0_ActualValues1.AMK_ActualVelocity);
    CANAL_PRINT("%sAMK_TorqueCurrent: %d", sep, AMK0_ActualValues1.AMK_TorqueCurrent);
    CANAL_PRINT("%sAMK_MagnetizingCurrent: %d", sep, AMK0_ActualValues1.AMK_MagnetizingCurrent);
    CANAL_PRINT("\n\r");
}
void Print_AMK0_ActualValues2() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: AMK0_ActualValues2");
    CANAL_PRINT("%sAMK_TempMotor: %f", sep, AMK0_ActualValues2.AMK_TempMotor);
    CANAL_PRINT("%sAMK_TempInverter: %f", sep, AMK0_ActualValues2.AMK_TempInverter);
    CANAL_PRINT("%sAMK_ErrorInfo: %u", sep, AMK0_ActualValues2.AMK_ErrorInfo);
    CANAL_PRINT("%sAMK_TempIGBT: %f", sep, AMK0_ActualValues2.AMK_TempIGBT);
    CANAL_PRINT("\n\r");
}
void Print_AMK0_SetPoints1() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: AMK0_SetPoints1");
    CANAL_PRINT("%sAMK_bInverterOn: %u", sep, AMK0_SetPoints1.AMK_bInverterOn);
    CANAL_PRINT("%sAMK_bDcOn: %u", sep, AMK0_SetPoints1.AMK_bDcOn);
    CANAL_PRINT("%sAMK_bEnable: %u", sep, AMK0_SetPoints1.AMK_bEnable);
    CANAL_PRINT("%sAMK_bErrorReset: %u", sep, AMK0_SetPoints1.AMK_bErrorReset);
    CANAL_PRINT("%sAMK_TargetVelocity: %d", sep, AMK0_SetPoints1.AMK_TargetVelocity);
    CANAL_PRINT("%sAMK_TorqueLimitPositiv: %d", sep, AMK0_SetPoints1.AMK_TorqueLimitPositiv);
    CANAL_PRINT("%sAMK_TorqueLimitNegativ: %d", sep, AMK0_SetPoints1.AMK_TorqueLimitNegativ);
    CANAL_PRINT("\n\r");
}
void Print_Contactor_States() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: Contactor_States");
    CANAL_PRINT("%sPack_Positive: %u", sep, Contactor_States.Pack_Positive);
    CANAL_PRINT("%sPack_Precharge: %u", sep, Contactor_States.Pack_Precharge);
    CANAL_PRINT("%sPack_Negative: %u", sep, Contactor_States.Pack_Negative);
    CANAL_PRINT("\n\r");
}
void Print_Pack_State() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: Pack_State");
    CANAL_PRINT("%sPack_Current: %f", sep, Pack_State.Pack_Current);
    CANAL_PRINT("%sPack_Inst_Voltage: %f", sep, Pack_State.Pack_Inst_Voltage);
    CANAL_PRINT("%sAvg_Cell_Voltage: %f", sep, Pack_State.Avg_Cell_Voltage);
    CANAL_PRINT("%sPopulated_Cells: %u", sep, Pack_State.Populated_Cells);
    CANAL_PRINT("\n\r");
}
void Print_Pack_Current_Limits() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: Pack_Current_Limits");
    CANAL_PRINT("%sPack_CCL: %u", sep, Pack_Current_Limits.Pack_CCL);
    CANAL_PRINT("%sPack_DCL: %u", sep, Pack_Current_Limits.Pack_DCL);
    CANAL_PRINT("\n\r");
}
void Print_Pack_SOC() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: Pack_SOC");
    CANAL_PRINT("%sPack_SOC: %f", sep, Pack_SOC.Pack_SOC);
    CANAL_PRINT("%sMaximum_Pack_Voltage: %f", sep, Pack_SOC.Maximum_Pack_Voltage);
    CANAL_PRINT("\n\r");
}
void Print_Contactor_Feedback() {
	char* sep ="\n\r\t";

	CANAL_PRINT("MESSAGE: Contactor_Feedback");
    CANAL_PRINT("%sPack_Precharge_Feedback: %u", sep, Contactor_Feedback.Pack_Precharge_Feedback);
    CANAL_PRINT("%sPack_Negative_Feedback: %u", sep, Contactor_Feedback.Pack_Negative_Feedback);
    CANAL_PRINT("%sPack_Positive_Feedback: %u", sep, Contactor_Feedback.Pack_Positive_Feedback);
    CANAL_PRINT("\n\r");
}

TeCanALRet Print_Message(uint32_t *ID) {
	CanALPrinter* print;

	for (int i = 0; i < TOTAL_MESSAGES; i++) {
		if ((*ID) == CANAL_PRINTER_TABLE[i].ID) {
			print = CANAL_PRINTER_TABLE[i].printer;

			(*print)();

			return CANAL_OK;
		}
	}

	return CANAL_UNSUPPORTED_TX_MESSAGE;
}

TeCanALRet GetTxDataLength(TeMessageID *ID, uint32_t *dlc) {
	for (int i = 0; i < NUM_TX_MESSAGES; i++) {
			if ((*ID) == CANAL_TX_MESSAGE_TABLE[i].ID) {
				*dlc = CANAL_TX_MESSAGE_TABLE[i].dlc;

				return CANAL_OK;
			}
		}

    return CANAL_UNSUPPORTED_TX_MESSAGE;
}

// UnmarshalBinary acts as a generic binary unmarshaller
TeCanALRet UnmarshalBinary(uint32_t *ID, uint8_t* rawData) {
	BinaryUnmarshaller* unmarshal;
	TeCanALRet ret;
 
	if ((ret = getBinaryUnmarshaller(ID, &unmarshal)) != CANAL_OK)
		return ret;
 
	return (*unmarshal)(rawData);
}

// MarshalBinary acts as a generic binary marshaller
TeCanALRet MarshalBinary(TeMessageID *ID, uint8_t* txData) {
	BinaryMarshaller* marshal;
	TeCanALRet ret;
 
	if ((ret = getBinaryMarshaller(ID, &marshal)) != CANAL_OK) return ret;
 
	return (*marshal)(txData);
}

