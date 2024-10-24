/*
 * USART_Cfg.h
 *
 *  Created on: Aug 24, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_USART_CFG_H_
#define INC_USART_CFG_H_

/*
 * OverSample16
 * OverSample8
 */
#define OVERSAMPLING_TYPE		OverSample16
/*
 *@ BAUDRATE 9600  = 104
 *@ BAUDRATE 115200	= 8
 */

#define MANTISSA_VALUE			 104

/*
 *@ BAUDRATE 9600  = 3
 *@ BAUDRATE 115200	= 11
 */

#define DIV_FRACTION_value		3

/*
 * STOP_1_DATA_8
 * STOP_1_DATA_9
 */

#define  WORDLENGTH 				STOP_1_DATA_8

/*
 * Parity_Enable
 * Parity_Disable
 */

#define PARITY_CONTROL					Parity_Disable

/*
 * Transmitter_Enable
 * Transmitter_Disable
 */

#define USART1_TRANSMITTER_STATE 		Transmitter_Enable

/*
 * Receiver_Enable
 * Receiver_Disable
 */
#define USART1_RECEIVER_STATE			Receiver_Enable

/*
 * Usart1_Enable
 * Usart1_Disable
 */
#define USART1_STATE 					Usart1_Enable

/***************USART6****************/
/*
 * OverSample16
 * OverSample8
 */
#define USART6_OVERSAMPLING_TYPE 	OverSample16
/*
 *@ BAUDRATE 9600  = 104
 *@ BAUDRATE 115200	= 8
 */
#define USART6_MANTISSA_VALUE		104
/*
 *@ BAUDRATE 9600  = 3
 *@ BAUDRATE 115200	= 11
 */
#define USART6_DIV_FRACTION_value	3
/*
 * STOP_1_DATA_8
 * STOP_1_DATA_9
 */
#define USART6_WORDLENGTH		 STOP_1_DATA_8
/*
 * Parity_Enable
 * Parity_Disable
 */

#define USART6_PARITY_CONTROL		Parity_Disable
/*
 * Transmitter_Enable
 * Transmitter_Disable
 */
#define USART6_TRANSMITTER_STATE	Transmitter_Enable
/*
 * Receiver_Enable
 * Receiver_Disable
 */
#define USART6_RECEIVER_STATE		Receiver_Enable
/*
 * Usart6_Enable
 * Usart6_Disable
 */
#define USART6_STATE				Usart6_Enable
#endif /* INC_USART_CFG_H_ */
