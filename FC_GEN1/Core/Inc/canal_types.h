/*
 * canal_types.h
 *
 *  Created on: Jan 22, 2023
 *      Author: Samuel Parent
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

#ifndef INC_CANAL_TYPES_H_
#define INC_CANAL_TYPES_H_

#define CANAL_DEBUG_MODE 0

#if CANAL_DEBUG_MODE
#define CANAL_PRINT printf
#else
#define CANAL_PRINT // Enable CANAL_DEBUG_MODE to print
#endif // CANAL_DEBUG_MODE

typedef enum {
	// CANAL_UNKNOWN_RETURN indicates an initialized return code
	CANAL_UNKOWN_RETURN = 0,
	// CANAL_OK indicates a successful function call
	CANAL_OK,
	// CANAL_NULL_REF indicates that the reference to TsCanAL is NULL
	CANAL_NULL_REF,
	// CANAL_CAN_HANDLE_NULL_REF indicates that the reference to CAN_HandleTypeDef is NULL
	CANAL_CAN_HANDLE_NULL_REF,
	// CANAL_UNSUPPORTED_INSTANCE indicates that the provided canNum is not supported
	CANAL_UNSUPPORTED_INSTANCE,
	// CAN_UNSUPPORTED_BAUD indicates that the provided baud rate is not supported
	CANAL_UNSUPPORTED_BAUD,
	// CANAL_UNSUPPORTED_MODE indicates that the selected can mode is not supported
	// by the canal library
	CANAL_UNSUPPORTED_MODE,
	// CANAL_UNSUPPORTED_PCLK1_FREQ indicates that the Peripheral Clock 1 frequency
	// is not in the CAN_BITRATE_TABLE (this is the clock that supports CAN 1-3).
	// To add entries for the a given freq go to: http://www.bittiming.can-wiki.info/
	CANAL_UNSUPPORTED_PCLK1_FREQ,
	// CANAL_UNSUPPORTED_RX_MESSAGE indicates that the message is not classified for
	// receiving
	CANAL_UNSUPPORTED_RX_MESSAGE,
	// CANAL_UNSUPPORTED_TX_MESSAGE indicates that the message is not classified for
	// transmission
	CANAL_UNSUPPORTED_TX_MESSAGE,
	// CANAL_INVALID_ENDIANNESS indicates that the given endianness was not little or big
	CANAL_INVALID_ENDIANNESS,
	// CAN_UNKOWN_IDE indicated that the RxHeader contains an unknown IDE (std vs ext ID)
	CANAL_UNKOWN_IDE,
	// CAN_INIT_FAILED indicates that the HAL_CAN_Init function returned !OK
	CANAL_INIT_FAILED,
	// CAN_START_FAILED indicates that the HAL_CAN_Start function returned !OK
	CANAL_START_FAILED,
	// CAN_CONFIG_FILTER_FAILED indicates the the HAL_CAN_ConfigFilter returned !OK
	CANAL_CONFIG_FILTER_FAILED,
	// CAN_GET_RXMESSAGE_FAILED indicates the HAL_CAN_GetRxMessage returned !OK
	CANAL_GET_RXMESSAGE_FAILED,
	// CAN_ERROR indicates a generic error has occurred
	CANAL_ERROR,
}TeCanALRet;

// TeEndianness contains endian type of incoming message
typedef enum {
    CANAL_UNKNOWN_ENDIANNESS = 0,
    CANAL_BIG_ENDIAN,
    CANAL_LITTLE_ENDIAN,
} TeCanALEndianness;

// BinaryUnmarshaller gets raw bytes from CAN Rx and updates the
// global struct instance with the associated CAN ID
typedef TeCanALRet BinaryUnmarshaller(uint8_t*);

// BinaryMarshaller takes the global struct instance and converts it to raw bytes
// to prepare it for transmission
typedef TeCanALRet BinaryMarshaller(uint8_t*);

// CanALPrinter will print the message associated wiht the CAN ID
typedef void CanALPrinter(void);

#endif /* INC_CANAL_TYPES_H_ */
