/*
 * MRCC_Prg.c
 *
 *  Created on: Jul 19, 2024
 *      Author: Mostafa Hisham
 *      Layer : MCAL
 */
#include<Std_Types.h>
#include<Macros.h>
#include<MRCC.h>
#include<MRCC_Prv.h>
#include<MRCC_Cfg.h>
/***************************************************************************************************************************/
				/*****A Function To Initialize The System Clock State*****/
/***************************************************************************************************************************/
void RCC_VInit(void)
{
	/*choose internal or external clock source for the system*/
#if	SYSTEM_CLK	== HSI_Clk
	SET_BIT(RCC->RCC_CR,HSI_ON);
	/*wait for flag */
	while(!GET_BIT(RCC->RCC_CR,HSI_RDY));		// while not equal one
	/*Select System Clock */
	RCC->RCC_CFGR = (RCC->RCC_CFGR & RCC_CFGR_MASK) | (SW_CLK);
#elif SYSTEM_CLK == HSE_CLk
	SET_BIT(RCC->RCC_CR,HSE_ON);
	/*wait for flag */
	while(!GET_BIT(RCC->RCC_CR,HSE_RDY));		// while not equal one
	/*Select System Clock*/
	RCC->RCC_CFGR = (RCC->RCC_CFGR & RCC_CFGR_MASK) | (SW_CLK);
#else
	#error "Undefined System Clk";
#endif
	//BYPASS PIN
#if HSE_TYPE == External_Clk
		CLR_BIT(RCC->RCC_CR,HSE_BYP);
#elif HSE_TYPE == RC_Clk
		SET_BIT(RCC->RCC_CR,HSE_BYP);
#else
	#error"Undefined Clk Type";
#endif
}
/***************************************************************************************************************************/
				/*****A Function To Enable The Peripherals Clock*****/
/***************************************************************************************************************************/
void RCC_VEnableClkPeripheral(Base_Type Copy_AddressBus,u8 Copy_u8Peripheral)
{
	switch(Copy_AddressBus)
		{
		case AHB1ENR:
			SET_BIT(RCC->RCC_AHB1ENR,Copy_u8Peripheral);
			break;
		case AHB2ENR:
			SET_BIT(RCC->RCC_AHB2ENR,Copy_u8Peripheral);
			break;
		case APB1ENR:
			SET_BIT(RCC->RCC_APB1ENR,Copy_u8Peripheral) ;
			break;
		case APB2ENR:
			SET_BIT(RCC->RCC_APB2ENR,Copy_u8Peripheral) ;
			break;
		case AHB1LPENR:
			SET_BIT(RCC->RCC_AHB1LPENR,Copy_u8Peripheral) ;
			break;
		case AHB2LPENR:
			SET_BIT(RCC->RCC_AHB2LPENR,Copy_u8Peripheral);
			break;
		case APB1LPENR:
			SET_BIT(RCC->RCC_APB1LPENR,Copy_u8Peripheral) ;
			break;
		case APB2LPENR:
			SET_BIT(RCC->RCC_APB2LPENR,Copy_u8Peripheral) ;
			break;
		default:
			break;

		}
}
/***************************************************************************************************************************/
				/*****A Function To Disable The Peripherals Clock*****/
/***************************************************************************************************************************/
void RCC_VDisableClkPeripheral(Base_Type Copy_AddressBus,u8 Copy_u8Peripheral)
{
	switch(Copy_AddressBus)
			{
			case AHB1ENR:
				CLR_BIT(RCC->RCC_AHB1ENR,Copy_u8Peripheral);
				break;
			case AHB2ENR:
				CLR_BIT(RCC->RCC_AHB2ENR,Copy_u8Peripheral);
				break;
			case APB1ENR:
				CLR_BIT(RCC->RCC_APB1ENR,Copy_u8Peripheral) ;
				break;
			case APB2ENR:
				CLR_BIT(RCC->RCC_APB2ENR,Copy_u8Peripheral) ;
				break;
			case AHB1LPENR:
				CLR_BIT(RCC->RCC_AHB1LPENR,Copy_u8Peripheral) ;
				break;
			case AHB2LPENR:
				CLR_BIT(RCC->RCC_AHB2LPENR,Copy_u8Peripheral);
				break;
			case APB1LPENR:
				CLR_BIT(RCC->RCC_APB1LPENR,Copy_u8Peripheral) ;
				break;
			case APB2LPENR:
				CLR_BIT(RCC->RCC_APB2LPENR,Copy_u8Peripheral) ;
				break;
			default:
				break;

			}

}



