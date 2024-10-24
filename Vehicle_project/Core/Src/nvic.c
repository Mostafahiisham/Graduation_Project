/*
 * nvic.c
 *
 *  Created on: Jul 27, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<NVIC.h>
#include<NVIC_Prv.h>

static u8 Copy_u8Global = 0;

u8 NVIC_EnableInterrupt(INTERRUPT_Position Copy_uddtPosition)
{
	u8 Local_u8ErrorState = OK;

	if(Copy_uddtPosition >= 0 && Copy_uddtPosition<=84)
	{
	NVIC->ISER[Copy_uddtPosition/NVIC_DIVISION] |= (1<<(Copy_uddtPosition % NVIC_REMINDER ));
	}
	else
	{
		Local_u8ErrorState = NOK;
	}
	return Local_u8ErrorState;
}

u8 NVIC_DisableInterrupt(INTERRUPT_Position Copy_uddtPosition)
{
	u8 Local_u8ErrorState = OK;

	if(Copy_uddtPosition >= 0 && Copy_uddtPosition<=84)
	{
	NVIC->ICER[Copy_uddtPosition/NVIC_DIVISION] |= (1<<(Copy_uddtPosition % NVIC_REMINDER ));
	}
	else
	{
		Local_u8ErrorState = NOK;
	}
	return Local_u8ErrorState;
}

u8 NVIC_SelectGroupNumber(NVIC_GROUP_Type Copy_uddtGroupType)
{
	u8 Local_u8ErrorState = OK;
	Copy_u8Global = Copy_uddtGroupType;
	u32 Local_u32Copy = 0;

	if(Copy_uddtGroupType >= 3 && Copy_uddtGroupType <= 7)
	{
		Local_u32Copy = VECTKEY |(Copy_uddtGroupType<<8);
		AIRCR  = Local_u32Copy;
	}
	else
	{
		Local_u8ErrorState = NOK;
	}
	return Local_u8ErrorState;
}
u8 NVIC_SetInterruptPriority(u8 Copy_u8GroupNum,u8 Copy_u8SubNum,INTERRUPT_Position Copy_uddtIntPos)
{
	u8 Local_u8ErrorState = OK;
	if(Copy_u8Global >= 3 && Copy_u8Global <=7)
	{
	switch(Copy_u8Global)
		{
		case GROUP16_SUB0:
			NVIC->IPR[Copy_uddtIntPos] = (Copy_u8GroupNum<<4);
			break;
		case GROUP8_SUB2:
			NVIC->IPR[Copy_uddtIntPos] = (Copy_u8GroupNum<<5)|(Copy_u8SubNum<<4);
			break;
		case GROUP4_SUB4:
			NVIC->IPR[Copy_uddtIntPos] = (Copy_u8GroupNum<<6)|(Copy_u8SubNum<<4);
			break;
		case GROUP2_SUB8:
			NVIC->IPR[Copy_uddtIntPos] = (Copy_u8GroupNum<<7)|(Copy_u8SubNum<<4);
			break;
		case GROUP0_SUB16:
			NVIC->IPR[Copy_uddtIntPos] = (Copy_u8SubNum<<4);
			break;
		default:
			break;
		}
	}
	else
	{
		Local_u8ErrorState = NOK;
	}
	return Local_u8ErrorState;
}





