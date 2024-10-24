/*
 * usart.c
 *
 *  Created on: Aug 24, 2024
 *      Author: Mostafa Hisham
 */
#include<STD_Types.h>
#include<Macros.h>
#include<USART.h>
#include<USART_Prv.h>
#include<USART_Cfg.h>
static void (*USART1_PTR[2])(void)={0};
static void (*USART6_PTR[2])(void)={0};
u8 transmitter;
void USART_Init(void)
{
	/*Select the USART1 OverSampling Type*/
#if OVERSAMPLING_TYPE == OverSample16
	USART1->CR1.OVER8 = 0;
#elif OVERSAMPLING == OverSample8
	USART1->CR1.OVER8 = 1;
#else
	#error"Undefined OverSample Type"
#endif
	/*Set the USART1 BaudRate*/
	USART1->BRR |=(MANTISSA_VALUE<<4)|(DIV_FRACTION_value);   	// MANTISSA 104 VALUE // DIV_FRACTION 3 VALUE

	/*USART1 Word Length*/
#if WORDLENGTH == STOP_1_DATA_8
	USART1->CR1.M = 0;
#elif WORDLENGTH == STOP_1_DATA_9
	USART1->CR1.M = 1;
#else
	#error "Undefined Word Length Type"
#endif

	/*The USART1 parity State*/
#if PARITY_CONTROL == Parity_Disable
	USART1->CR1.PCE = 0;
#elif PARITY_CONTROL == Parity_Enable
	USART1->CR1.PCE = 1;
#else
	#error"Undefined Parity Control State"
#endif

	/*USART1 Transmit  State*/
#if USART1_TRANSMITTER_STATE == Transmitter_Disable
	USART1->CR1.TE = 0;
#elif USART1_TRANSMITTER_STATE == Transmitter_Enable
	USART1->CR1.TE = 1;
#else
	#error"Undefined Transmitter State"
#endif

	/*USART1 Receiver State*/
#if USART1_RECEIVER_STATE == Receiver_Disable
	USART1->CR1.RE = 0;
#elif USART1_RECEIVER_STATE == Receiver_Enable
	USART1->CR1.RE = 1;
#else
	#error"Undefined Receiver State"
#endif

	/*USART1 State*/
#if USART1_STATE == Usart1_Disable
	USART1->CR1.UE = 0;
#elif USART1_STATE == Usart1_Enable
	USART1->CR1.UE = 1;
#else
	#error"Undefined USART1 State"
#endif

}
void USART6_Init(void)
{

	/*Select the USART6 OverSampling Type*/
#if USART6_OVERSAMPLING_TYPE == OverSample16
	USART6->CR1.OVER8 = 0;
#elif USART6_OVERSAMPLING_TYPE == OverSample8
	USART6->CR1.OVER8 = 1;
#else
	#error"Undefined OverSample Type"
#endif
	/*Set the USART6 BaudRate*/
	USART6->BRR |=(USART6_MANTISSA_VALUE<<4)|(USART6_DIV_FRACTION_value);   	// MANTISSA 104 VALUE // DIV_FRACTION 3 VALUE

	/*USART6 Word Length*/
#if USART6_WORDLENGTH == STOP_1_DATA_8
	USART6->CR1.M = 0;
#elif USART6_WORDLENGTH == STOP_1_DATA_9
	USART6->CR1.M = 1;
#else
	#error "Undefined Word Length Type"
#endif

	/*The USART6 parity State*/
#if USART6_PARITY_CONTROL == Parity_Disable
	USART6->CR1.PCE = 0;
#elif USART6_PARITY_CONTROL == Parity_Enable
	USART6->CR1.PCE = 1;
#else
	#error"Undefined Parity Control State"
#endif

	/*USART6 Transmit  State*/
#if USART6_TRANSMITTER_STATE == Transmitter_Disable
	USART6->CR1.TE = 0;
#elif USART6_TRANSMITTER_STATE == Transmitter_Enable
	USART6->CR1.TE = 1;
#else
	#error"Undefined Transmitter State"
#endif

	/*USART6 Receiver State*/
#if USART6_RECEIVER_STATE == Receiver_Disable
	USART6->CR1.RE = 0;
#elif USART6_RECEIVER_STATE == Receiver_Enable
	USART6->CR1.RE = 1;
#else
	#error"Undefined Receiver State"
#endif

	/*USART6 State*/
#if USART6_STATE == Usart6_Enable
	USART6->CR1.UE = 1;
#elif USART6_STATE == Usart6_Disable
	USART6->CR1.UE = 0;
#else
	#error"Undefined USART6 State"
#endif
}
void USART1_TX_InterruptState(USART_TX_Int_state TX_INT)
{
	switch(TX_INT)
	{
	case TX_INT_Disable:
		USART1->CR1.TXEIE = 0;
	break;
	case TX_INT_Enable:
		USART1->CR1.TXEIE = 1;
	break;
	default:
		break;
	}
}
void USART6_TX_InterruptState(USART_TX_Int_state TX_INT)
{
	switch(TX_INT)
	{
	case TX_INT_Disable:
		USART6->CR1.TXEIE = 0;
	break;
	case TX_INT_Enable:
		USART6->CR1.TXEIE = 1;
	break;
	default:
		break;
	}
}
void USART1_RX_InterruptState(USART_RX_Int_state RX_INT)
{
	switch(RX_INT)
	{
	case RX_INT_Disable:
		USART1->CR1.RXNEIE = 0;
	break;
	case RX_INT_Enable:
		USART1->CR1.RXNEIE = 1;
	break;

	}
}
void USART6_RX_InterruptState(USART_RX_Int_state RX_INT)
{
	switch(RX_INT)
	{
	case RX_INT_Disable:
		USART6->CR1.RXNEIE = 0;
	break;
	case RX_INT_Enable:
		USART6->CR1.RXNEIE = 1;
	break;

	}
}
void USART_SendData(u8 Copy_u8Data)
{
	/*Check the Transmitted data Register is Empty or not by getting the flag value*/
	while(!GET_BIT(USART1->SR,SR_TXE));

	/*Enter the Data to the register*/
	USART1->DR = Copy_u8Data;
}
void USART6_SendData(u8 Copy_u8Data)
{
	/*Check the Transmitted data Register is Empty or not by getting the flag value*/
	while(!GET_BIT(USART6->SR,SR_TXE));

	/*Enter the Data to the register*/
	USART6->DR = Copy_u8Data;
}

u8 USART_ReceiveData(void)
{

	/*Check the Received data Register is Empty or not by getting the flag value*/
	while(!GET_BIT(USART1->SR,SR_RXNE));
	/*Read the Sent Data*/
	return USART1->DR;
}
u8 USART6_ReceiveData(void)
{

	/*Check the Received data Register is Empty or not by getting the flag value*/
	while(!GET_BIT(USART6->SR,SR_RXNE));
	/*Read the Sent Data*/
	return USART6->DR;
}
void USART_SendString(u8 *Str)
{
	u32 i =0;
	while(Str[i] != '\0')
	{
		USART_SendData(Str[i]);
		i++;
	}
}
void USART6_SendString(u8 *Str)
{
	u32 i =0;
	while(Str[i] != '\0')
	{
		USART6_SendData(Str[i]);
		i++;
	}
}
u8 USART_ReceiveString(void)
{	u8 * Str = 0 ;
	u32 i = 0;
	while(Str[i] != '\0')
	{
		Str[i] = USART_ReceiveData();
		i++;
	}
}

u8 USART6_ReceiveString(void)
{	u8 * Str = 0 ;
	u32 i = 0;
	while(Str[i] != '\0')
	{
		Str[i] = USART6_ReceiveData();
		i++;
	}
}

void USART1_TransmitArray(u8 *Ptr_u8Data,u32 Arr_u32Length)
{
	if(*Ptr_u8Data != NULL)
	{
		for(u8 Counter = 0;Counter<Arr_u32Length;Counter++)
		{
			USART_SendData(Ptr_u8Data[Counter]);
		}
	}
}
void USART6_TransmitArray(u8 *Ptr_u8Data,u32 Arr_u32Length)
{
	if(*Ptr_u8Data != NULL)
	{
		for(u8 Counter = 0;Counter<Arr_u32Length;Counter++)
		{
			USART6_SendData(Ptr_u8Data[Counter]);
		}
	}
}
void USART1_RecieveArray(u8 *ptr_u8Data,u32 Arr_u32Length)
{
	for(u8 Counter =0;Counter < Arr_u32Length;Counter++)
	{
		ptr_u8Data[Counter] = USART_ReceiveData();
	}
}
void USART6_RecieveArray(u8 *ptr_u8Data,u32 Arr_u32Length)
{
	for(u8 Counter =0;Counter < Arr_u32Length;Counter++)
	{
		ptr_u8Data[Counter] = USART6_ReceiveData();
	}
}

u8 USART1_ReadDataRegister(void)
{
	return USART1->DR;
}
u8 USART6_ReadDataRegister(void)
{
	return USART6->DR;
}

void USART1_SendDataRegister(u8 Data)
{
	 USART1->DR =Data;
}
void USART6_SendDataRegister(u8 Data)
{
	 USART6->DR =Data;
}

void USART1_SendNumber(s64 S_Number)
{
	u64 u_Number = 1;
	if (S_Number == 0)
	{
		USART_SendData('0');
	}
	if(S_Number < 0)
	{
		USART_SendData('0');
		S_Number *=-1;
	}
	while(S_Number != 0 )
	{
		u_Number = ((u_Number * 10)+(S_Number % 10));
		S_Number = S_Number / 10;
	}
	while(u_Number != 1)
	{
		USART_SendData((u_Number%10)+48);
		u_Number =u_Number /10;
	}
}
void USART6_SendNumber(s64 S_Number)
{
	u64 u_Number = 1;
	if (S_Number == 0)
	{
		USART6_SendData('0');
	}
	if(S_Number < 0)
	{
		USART6_SendData('0');
		S_Number *=-1;
	}
	while(S_Number != 0 )
	{
		u_Number = ((u_Number * 10)+(S_Number % 10));
		S_Number = S_Number / 10;
	}
	while(u_Number != 1)
	{
		USART6_SendData((u_Number%10)+48);
		u_Number =u_Number /10;
	}
}
void USART1_CallBack(void(*ptr)(void),USART_INT_t Int_Type)
{
	switch(Int_Type)
	{
	case TX_Interrupt:
		USART1_PTR[0] = ptr;

	break;
	case RX_Interrupt:
		USART1_PTR[1]= ptr;

	break;
	}
}
void USART6_CallBack(void(*ptr)(void),USART_INT_t Int_Type)
{
	switch(Int_Type)
	{
	case TX_Interrupt:
		USART6_PTR[0] = ptr;

	break;
	case RX_Interrupt:
		USART6_PTR[1]= ptr;

	break;
	}
}
void USART6_Transmitdata(u8 data)
{
	transmitter =data;
}
void USART6_SendNumberINT(s64 S_Number)
{
	u64 u_Number = 1;
	if (S_Number == 0)
	{
		USART6_Transmitdata('0');
	}
	if(S_Number < 0)
	{
		USART6_Transmitdata('0');
		S_Number *=-1;
	}
	while(S_Number != 0 )
	{
		u_Number = ((u_Number * 10)+(S_Number % 10));
		S_Number = S_Number / 10;
	}
	while(u_Number != 1)
	{
		USART6_Transmitdata((u_Number%10)+48);
		u_Number =u_Number /10;
	}
}
void USART6_SendStringInt(u8 *Str)
{
	u32 i =0;
		while(Str[i] != '\0')
		{
			USART6_Transmitdata(Str[i]);
			i++;
		}
}
void USART1_IRQHandler(void)
{
	u8 temp =0;
/*
	if(GET_BIT(USART1->SR,SR_TC)==1)
	{
		USART1_PTR[0]();
		USART1->DR = temp;
	}
	*/
	if(GET_BIT(USART1->SR,SR_RXNE)==1)
	{
		USART1_PTR[1]();
		temp = USART1->DR;
	}

}

void USART6_IRQHandler(void)
{
	u8 temp = 0 ;
	if(GET_BIT(USART6->SR,SR_TXE)==1)
	{
		USART6->DR =transmitter;
	}
	/*
	if(GET_BIT(USART6->SR,SR_RXNE)==1)
	{
		USART6_PTR[1]();
		temp = USART6->DR;
	}
	*/
}
