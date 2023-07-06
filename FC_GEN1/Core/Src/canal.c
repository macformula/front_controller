/*
 * canal.c
 *
 *  Created on: Jan 22, 2023
 *      Author: Samuel Parent
 *
 *                   ..::^~~~!!~~~^^:..
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
*                       INCLUDES
*********************************************************/

#include "canal.h"

/*********************************************************
*                       HELPERS
*********************************************************/

// This table exists as the there are multiple solutions for timing to the set of
// parameters: {PCLK1_Freq, Baud}. There are are preferred solutions, which
// can be found here: http://www.bittiming.can-wiki.info/
static const struct {
	uint8_t pclk1Mhz;
	uint16_t baudKb;
	uint8_t prescaler;
	uint32_t seg1;
	uint32_t seg2;
}CAN_BITRATE_TABLE[NUM_BITRATE_ENTRIES] = {
	{16,    1000,    1,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
	{16,     500,    2,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
	{16,     250,    4,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
	{16,     100,   10,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
	{8,	    1000,    1,    CAN_BS1_6TQ,     CAN_BS2_1TQ},
	{8,	     500,    1,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
	{8,	     250,    2,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
	{8,      100,    5,    CAN_BS1_13TQ,    CAN_BS2_2TQ},
};


static TeCanALRet setInstance(CAN_HandleTypeDef* hcan, TeCanALInstance canNum){
	switch (canNum) {
	case CANAL_INST_CAN_1:
		hcan->Instance = CAN1;
		break;
	case CANAL_INST_CAN_2:
		hcan->Instance = CAN2;
		break;
	case CANAL_INST_CAN_3:
		hcan->Instance = CAN3;
		break;
	default:
		hcan->Instance = NULL;
		return CANAL_UNSUPPORTED_INSTANCE;
	}

	return CANAL_OK;
}

static void setDefaults(CAN_HandleTypeDef* hcan) {
	hcan->Init.TimeTriggeredMode = DISABLE;
	hcan->Init.AutoBusOff = ENABLE;
	hcan->Init.AutoWakeUp = DISABLE;
	hcan->Init.AutoRetransmission = DISABLE;
	hcan->Init.ReceiveFifoLocked = DISABLE;
	hcan->Init.TransmitFifoPriority = DISABLE;
}


static TeCanALRet setTimingParams(CAN_HandleTypeDef* hcan, TeCanALBaud baud) {
    uint8_t freq = (HAL_RCC_GetPCLK1Freq() / FREQ_MHZ);

    if (!IS_CANAL_BAUDRATE(baud)) return CANAL_UNSUPPORTED_BAUD;

    for (uint8_t i = 0; i < NUM_BITRATE_ENTRIES; i++) {
        if (freq == CAN_BITRATE_TABLE[i].pclk1Mhz) {
            if (baud == CAN_BITRATE_TABLE[i].baudKb){
                hcan->Init.Prescaler  = CAN_BITRATE_TABLE[i].prescaler;
                hcan->Init.TimeSeg1   = CAN_BITRATE_TABLE[i].seg1;
                hcan->Init.TimeSeg2   = CAN_BITRATE_TABLE[i].seg2;
                hcan->Init.SyncJumpWidth = DEFAULT_SYNC_JW;

                return CANAL_OK;
            }
        }
    }

    return CANAL_UNSUPPORTED_PCLK1_FREQ;
}

static TeCanALRet setMode(CAN_HandleTypeDef* hcan, TeCanALMode mode) {
	if (!IS_CANAL_MODE(mode)) return CANAL_UNSUPPORTED_MODE;

	hcan->Init.Mode = mode;

	return CANAL_OK;
}

static TeCanALRet setFilters(CAN_HandleTypeDef* hcan) {
	// TODO: Do we have any use for this?
	CAN_FilterTypeDef filterConfig = {
		.FilterBank = DEFAULT_FILTER_BANK,
		.FilterMode = DEFAULT_FILTER_MODE,
		.FilterScale = DEFAULT_FILTER_SCALE,
		.FilterIdHigh = DEFAULT_FILTER_ID_HIGH,
		.FilterIdLow = DEFAULT_FILTER_ID_LOW,
		.FilterMaskIdHigh = DEFAULT_FILTER_MASK_ID_HIGH,
		.FilterMaskIdLow = DEFAULT_FILTER_MASK_ID_LOW,
		.FilterFIFOAssignment = DEFAULT_RX_FIFO,
		.FilterActivation = DEFAULT_FILTER_ACTIVATION,
		.SlaveStartFilterBank = DEFAULT_FILTER_SLAVE_START_BANK,
	};

	if (HAL_CAN_ConfigFilter(hcan, &filterConfig) != HAL_OK){
		return CANAL_CONFIG_FILTER_FAILED;
	}

	return CANAL_OK;
}

/*********************************************************
*              PUBLIC FUNCTION DEFINITIONS
*********************************************************/

// Can_Init initializes CAN hardware and must be called with successful return
// before any other CAN functions
TeCanALRet CanAL_Init(TsCanAL* can) {
	TeCanALRet ret = CANAL_OK;

	if (can == NULL) return CANAL_NULL_REF;

	if (can->hcan == NULL) return CANAL_CAN_HANDLE_NULL_REF;

	if ((ret = setInstance(can->hcan, can->canNum)) != CANAL_OK) return ret;

	if ((ret = setTimingParams(can->hcan, can->baud)) != CANAL_OK) return ret;

	if ((ret = setMode(can->hcan, can->mode)) != CANAL_OK) return ret;

	setDefaults(can->hcan);

	if (HAL_CAN_Init(can->hcan) != HAL_OK) return CANAL_INIT_FAILED;

	if ((ret = setFilters(can->hcan)) != CANAL_OK) return ret;

	if (HAL_CAN_Start(can->hcan) != HAL_OK) return CANAL_START_FAILED;

	if (HAL_CAN_ActivateNotification(can->hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
		!= HAL_OK) return CANAL_ERROR;

	return ret;
}



TeCanALRet CanAL_Receive(TsCanAL* can) {
	TeCanALRet ret;
	CAN_RxHeaderTypeDef RxHeader = {0};
	uint8_t RxData[8] = {0};
	uint32_t ID;

	if (can == NULL) return CANAL_NULL_REF;

	// Get raw message
	if (HAL_CAN_GetRxMessage(can->hcan, DEFAULT_RX_FIFO, &RxHeader, RxData) != HAL_OK) {
		return CANAL_GET_RXMESSAGE_FAILED;
	}

	// Message ID can either be standard or extended
	switch (RxHeader.IDE) {
		case CAN_ID_STD:
			ID = RxHeader.StdId;
			break;
		case CAN_ID_EXT:
			ID = RxHeader.ExtId;
			break;
		default:
			return CANAL_UNKOWN_IDE;
	}

	if ((ret = UnmarshalBinary(&ID, RxData)) != CANAL_OK) return ret;

	return Print_Message(&ID);
}

TeCanALRet CanAL_Transmit(TsCanAL* can, TeMessageID ID) {
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t TxBuffer[8] = {0};
	uint32_t TxMailbox = 0;
	TeCanALRet ret;
	uint32_t data_len;

	if ((ret = GetTxDataLength(&ID, &data_len)) != CANAL_OK) return ret;

	TxHeader.DLC = data_len;

	if (IS_CAN_STDID(ID)) {
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.StdId = ID;
	}
	else if(IS_CAN_EXTID(ID)) {
		TxHeader.IDE = CAN_ID_EXT;
		TxHeader.ExtId = ID;
	}
	else {
		return CANAL_UNSUPPORTED_TX_MESSAGE;
	}

	// Data frame
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;

	MarshalBinary(&ID, TxBuffer);

	HAL_CAN_AddTxMessage(can->hcan, &TxHeader, TxBuffer, &TxMailbox);

	return CANAL_OK;
}


