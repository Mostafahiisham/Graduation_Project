/*
 * USART.h
 *
 *  Created on: Aug 24, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_USART_H_
#define INC_USART_H_

/*Over Sampling Types*/
#define OverSample16			1
#define OverSample8				2
/*Word Length Types*/
#define STOP_1_DATA_8			1
#define STOP_1_DATA_9			2
/*Parity Control State*/
#define Parity_Disable			1
#define Parity_Enable			2
/*usart1 Transmitter state type*/
#define Transmitter_Disable 	1
#define Transmitter_Enable		2
/*USART1 Receiver state type*/
#define Receiver_Disable		1
#define Receiver_Enable			2
/*USART1 State*/
#define Usart1_Disable			1
#define Usart1_Enable			2

typedef enum{
	TX_INT_Disable,
	TX_INT_Enable
}USART_TX_Int_state;

typedef enum{
	RX_INT_Disable	,
	RX_INT_Enable
}USART_RX_Int_state;
typedef enum{
	TX_Interrupt,
	RX_Interrupt
}USART_INT_t;

void USART_Init(void);
void USART6_Init(void);

void USART1_TX_InterruptState(USART_TX_Int_state TX_INT);
void USART6_TX_InterruptState(USART_TX_Int_state TX_INT);

void USART1_RX_InterruptState(USART_RX_Int_state RX_INT);
void USART6_RX_InterruptState(USART_RX_Int_state RX_INT);

void USART_SendData(u8 Copy_u8Data);
void USART6_SendData(u8 Copy_u8Data);

u8 USART_ReceiveData(void);
u8 USART6_ReceiveData(void);

void USART_SendString(u8 *Str);
void USART6_SendString(u8 *Str);

u8 USART_ReceiveString(void);
u8 USART6_ReceiveString(void);

void USART1_TransmitArray(u8 *Ptr_u8Data,u32 Arr_u32Length);
void USART6_TransmitArray(u8 *Ptr_u8Data,u32 Arr_u32Length);

void USART1_RecieveArray(u8 *ptr_u8Data,u32 Arr_u32Length);
void USART6_RecieveArray(u8 *ptr_u8Data,u32 Arr_u32Length);

u8 USART1_TX_ReadFlag(void);

u8 USART1_RX_ReadFlag(void);

void USART1_SendDataRegister(u8 Data);
void USART6_SendDataRegister(u8 Data);

u8 USART1_ReadDataRegister(void);
u8 USART6_ReadDataRegister(void);

void USART1_SendNumber(s64 S_Number);
void USART6_SendNumber(s64 S_Number);
void USART6_Transmitdata(u8 data);
void USART6_SendNumberINT(s64 S_Number);
void USART6_SendStringInt(u8 *str);
void USART1_CallBack(void(*ptr)(void),USART_INT_t Int_Type);
void USART6_CallBack(void(*ptr)(void),USART_INT_t Int_Type);


#endif /* INC_USART_H_ */
