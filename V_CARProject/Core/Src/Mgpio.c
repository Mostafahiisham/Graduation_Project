/*
 * Mgpio.c
 *
 *  Created on: Jul 20, 2024
 *      Author: Mostafa Hisham
 */
#include"Std_Types.h"
#include"Macros.h"
#include"MGPIO_Prv.h"
#include"MGPIO.h"

/***************************************************************************************************************************/
						/*****This Function To Set The GPIO Port Mode*****/
/***************************************************************************************************************************/
void GPIO_SetMode(PORT_t Port,PIN_Number Pin,Mode_t Mode)
{

	switch(Port)
	{
	case PORTA:
		GPIOA->MODER	&=~ (MODE_MUSK <<(Pin*MODE_BIT_ACCESS));		// BIT MASK
		GPIOA->MODER  	|=  (Mode<<(Pin*MODE_BIT_ACCESS));				//set the bit mode
		break;
	case PORTB:
		GPIOB->MODER	&=~ (MODE_MUSK <<(Pin*MODE_BIT_ACCESS));
		GPIOB->MODER  	|=  (Mode<<(Pin*MODE_BIT_ACCESS));
		break;
	case PORTC:
		GPIOC->MODER	&=~ (MODE_MUSK <<(Pin*MODE_BIT_ACCESS));
		GPIOC->MODER  	|=  (Mode<<(Pin*MODE_BIT_ACCESS));
		break;
	default:
		break;


	}

}
/***************************************************************************************************************************/
		/*****This Function To Set The Output GPIO Port Pin Mode  Open Drain , The Default(push_pull)*****/
/***************************************************************************************************************************/
void GPIO_SetOutputMode(PORT_t Port,PIN_Number Pin,OUTPUT_t Omode)
{

	switch(Port)
	{
	case PORTA:
		GPIOA->OTYPER	&=~ (1<<(Pin));			//clear bits
		GPIOA->OTYPER 	|=  (Omode<<(Pin));				//set the bit mode
		break;
	case PORTB:
		GPIOB->OTYPER	&=~ (1<<(Pin));
		GPIOB->OTYPER 	|=  (Omode<<(Pin));
		break;
	case PORTC:
		GPIOC->OTYPER	&=~ (1<<(Pin));
		GPIOC->OTYPER  	|=  (Omode<<(Pin));
		break;
	default:
		break;
	}
}
/***************************************************************************************************************************/
		/*****To Set The Output GPIO Port Pin Speed LOW SPEED, MEDIUM ,HIGH_SPEED,VERY_HIGH_SPEED*****/
/***************************************************************************************************************************/
void GPIO_SetOutputSpeed(PORT_t Port,PIN_Number Pin,OUTPUTSpeed_t O_Speed)
{
	switch(Port)
	{
	case PORTA:
		GPIOA->OSPEEDR	&=~ (SPEED_MUSK <<(Pin*SPEED_BIT_ACCESS));			//clear bits
		GPIOA->OSPEEDR 	|=  (O_Speed<<(Pin*SPEED_BIT_ACCESS));				//set the bit mode
		break;
	case PORTB:
		GPIOB->OSPEEDR	&=~ (SPEED_MUSK  <<(Pin*SPEED_BIT_ACCESS));
		GPIOB->OSPEEDR  |=  (O_Speed<<(Pin*SPEED_BIT_ACCESS));
		break;
	case PORTC:
		GPIOC->OSPEEDR	&=~ (SPEED_MUSK  <<(Pin*SPEED_BIT_ACCESS));
		GPIOC->OSPEEDR  |=  (O_Speed<<(Pin*SPEED_BIT_ACCESS));
		break;
	default:
		break;


	}
}
/***************************************************************************************************************************/
							/*****To Set The GPIO Port Pin Value Is High Or Low*****/
/***************************************************************************************************************************/
void GPIO_SetPinValue(PORT_t Port,PIN_Number Pin,PIN_Value P_Value)
{
	switch(Port)
		{
		case PORTA:
			GPIOA->ODR	&=~ (1<<(Pin));			//clear bits
			GPIOA->ODR 	|=  (P_Value<<(Pin));				//set the bit mode
			break;
		case PORTB:
			GPIOB->ODR	&=~ (1<<(Pin));
			GPIOB->ODR 	|=  (P_Value<<(Pin));
			break;
		case PORTC:
			GPIOC->ODR	&=~ (1<<(Pin));
			GPIOC->ODR  |=  (P_Value<<(Pin));
			break;
		default:
			break;
		}
}
/***************************************************************************************************************************/
				/******To Set The GPIO Port Pin Pull Up Or Pull Down*****/
/***************************************************************************************************************************/
void GPIO_SetPullType(PORT_t Port,PIN_Number Pin,PULL_t PULL_Type)
{

	switch (Port)
	{
	case PORTA:
		GPIOA->PUPDR &=~(PUPDR_MUSK<<(Pin*PUPDR_BIT_ACCESS));
		GPIOA->PUPDR |= (PULL_Type<<(Pin*PUPDR_BIT_ACCESS));
		break;
	case PORTB:
		GPIOB->PUPDR &=~(PUPDR_MUSK<<(Pin*PUPDR_BIT_ACCESS));
		GPIOB->PUPDR |= (PULL_Type<<(Pin*PUPDR_BIT_ACCESS));
		break;
	case PORTC:
		GPIOC->PUPDR &=~(PUPDR_MUSK<<(Pin*PUPDR_BIT_ACCESS));
		GPIOC->PUPDR |= (PULL_Type<<(Pin*PUPDR_BIT_ACCESS));
		break;
	default:
		break;

	}
}
/***************************************************************************************************************************/
				/*****To Read The GPIO Port Pin Value On The Input Pin*****/
/***************************************************************************************************************************/
void GPIO_GetPinValue(PORT_t Port,PIN_Number Pin,u32 *copy_PinValue)
{
	switch(Port)
		{

		case PORTA:
			 *copy_PinValue = GET_BIT(GPIOA->IDR,Pin);
			break;
		case PORTB:
			 *copy_PinValue = GET_BIT(GPIOB->IDR,Pin);
				break;
		case PORTC:
			 *copy_PinValue = GET_BIT(GPIOC->IDR,Pin);
				break;
		default:
			break;
		}
	return  *copy_PinValue;
}
/***************************************************************************************************************************/
					/*****To Toggle GPIO Port Pin Value*****/
/***************************************************************************************************************************/
void GPIO_TogglePinValue(PORT_t Port,PIN_Number Pin)
{
	switch(Port)
	{
	case PORTA:
		TOGGLE_BIT(GPIOA->ODR,Pin);
		break;
	case PORTB:
		TOGGLE_BIT(GPIOB->ODR,Pin);
			break;
	case PORTC:
		TOGGLE_BIT(GPIOC->ODR,Pin);
			break;
	}
}

/***************************************************************************************************************************/
							/*****To Set The GPIO Port Level*****/
/***************************************************************************************************************************/
void GPIO_SetPortValue(PORT_t Port,PORT_Value P_Value)
{
	switch(Port)
	{
	case PORTA:
		GPIOA->ODR = P_Value;
		break;
	case PORTB:
		GPIOB->ODR = P_Value;
		break;
	case PORTC:
		GPIOC->ODR = P_Value;
		break;
	}
}
/***************************************************************************************************************************/
						/*****To Set The Whole Port Mode*****/
/***************************************************************************************************************************/
void GPIO_SetPortMode(PORT_t Port,PORT_Mode P_Mode)
{
	switch(Port)
	{
	case PORTA:
		GPIOA->MODER = P_Mode;
		break;
	case PORTB:
		GPIOB->MODER =  P_Mode;
		break;
	case PORTC:
		GPIOC->MODER =  P_Mode;
		break;
	}

}
/***************************************************************************************************************************/
					/****To Set The GPIO Port Pin Value with Atomic Instruction *****/
/***************************************************************************************************************************/
void GPIO_SetAtomicPinValue(PORT_t Port,PIN_Number Pin,PIN_Value P_Value)
{
	switch(Port)
	{
	case PORTA:
		if(P_Value == PIN_HIGH)
		{
			GPIOA->BSRR &=~(1<<Pin);
			GPIOA->BSRR |=(1<<Pin);

		}
		else if(P_Value == PIN_LOW)
		{
			GPIOA->BSRR &=~(1<<(Pin+16));
			GPIOA->BSRR |=(1<<(Pin+16));
		}
		else
		{
			//Do Nothing
		}
		break;
	case PORTB:
			if(P_Value == PIN_HIGH)
			{
				GPIOB->BSRR &=~(1<<Pin);
				GPIOB->BSRR |=(1<<Pin);

			}
			else if(P_Value == PIN_LOW)
			{
				GPIOB->BSRR &=~(1<<(Pin+16));
				GPIOB->BSRR |=(1<<(Pin+16));
			}
			else
			{
				//Do Nothing
			}
			break;
	case PORTC:
			if(P_Value == PIN_HIGH)
			{
				GPIOC->BSRR &=~(1<<Pin);
				GPIOC->BSRR |=(1<<Pin);

			}
			else if(P_Value == PIN_LOW)
			{
				GPIOC->BSRR &=~(1<<(Pin+16));
				GPIOC->BSRR |=(1<<(Pin+16));
			}
			else
			{
				//Do Nothing
			}
			break;
	default:
		break;
	}
}
/***************************************************************************************************************************/
					/****To Set The GPIO Port Pin ALTERNATE FUNCTION MODE *****/
/***************************************************************************************************************************/
void GPIO_SetAlternateFunc(PORT_t Port,PIN_Number Pin,AlterFunc_Number AF_Number)
{
	u8 REG_Number = Pin / 8;
	u8 Bit_Number = (Pin % 8);
	switch(Port)
	{
	case PORTA:
		GPIOA->AFR[REG_Number] &=~(AFR_MUSK<<Bit_Number*AFR_BIT_ACCESS);
		GPIOA->AFR[REG_Number] |=(AF_Number<<Bit_Number*AFR_BIT_ACCESS);
		break;
	case PORTB:
		GPIOB->AFR[REG_Number] &=~(AFR_MUSK<<Bit_Number*AFR_BIT_ACCESS);
		GPIOB->AFR[REG_Number] |=( AF_Number<<Bit_Number*AFR_BIT_ACCESS);
		break;
	case PORTC:
		GPIOC->AFR[REG_Number] &=~(AFR_MUSK<<Bit_Number*AFR_BIT_ACCESS);
		GPIOC->AFR[REG_Number] |=( AF_Number<<Bit_Number*AFR_BIT_ACCESS);
		break;
	}
}
