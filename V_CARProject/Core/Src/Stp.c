/*
 * Stp.c
 *
 *  Created on: Aug 23, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<MGPIO.h>
#include<SYSTICK.h>
#include<STP.h>
#include<STP_Cfg.h>

void STP_Init(void)
{
	GPIO_SetMode(STP_PORT,SHIFT_PIN,OUTPUT);
	GPIO_SetOutputMode(STP_PORT,SHIFT_PIN,PUSH_PULL);

	GPIO_SetMode(STP_PORT,SERIAL_INPUT_PIN,OUTPUT);
	GPIO_SetOutputMode(STP_PORT,SERIAL_INPUT_PIN,PUSH_PULL);

	GPIO_SetMode(STP_PORT,LATCH_PIN,OUTPUT);
	GPIO_SetOutputMode(STP_PORT,LATCH_PIN,PUSH_PULL);
}
void STP_ShiftData(u8 Copy_u8Data)
{
	for(u8 i = 0;i < 8;i++)
	{
	GPIO_SetPinValue(STP_PORT,SERIAL_INPUT_PIN,GET_BIT(Copy_u8Data,i));
	GPIO_SetPinValue(STP_PORT,SHIFT_PIN,PIN_LOW);

	SYSTICK_Delay_ms(1);
	GPIO_SetPinValue(STP_PORT,SHIFT_PIN,PIN_HIGH);
	}

}
void STP_SendData(void)
{
	GPIO_SetPinValue(STP_PORT,LATCH_PIN,PIN_LOW);
	SYSTICK_Delay_ms(1);
	GPIO_SetPinValue(STP_PORT,LATCH_PIN,PIN_HIGH);
}

