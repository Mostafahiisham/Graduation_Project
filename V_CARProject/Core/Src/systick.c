/*
 * systick.c
 *
 *  Created on: Aug 2, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<SYSTICK.h>
#include<SYSTICK_Cfg.h>
#include<SYSTICK_Prv.h>

static void (*SYSTICK_PTR) (void) = NULL;
/***************************************************************************************************************************/
					/*****This Function To Initialize the Systick Clock scale and Interrupt State*****/
/***************************************************************************************************************************/
void SYSTICK_Init(void)
{

#if CLK_SOURCE == CLK_SOURCE_AHB_8
	CLR_BIT(SYSTICK->CTRL,CLKSOURCE);
#elif CLK_SOURCE == CLK_SOURCE_AHB
	SET_BIT(SYSTICK->CTRL,CLKSOURCE);
#else
	#error"Undefined Clk Source"
#endif
						/*To Control Interrupt State for PRE-Build Configuration*/
#if TICK_INTERRUPT == SYSTICK_INT_ENABLE
		SET_BIT(SYSTICK->CTRL,TICKINT);
#elif TICK_INTERRUPT == SYSTICK_INT_DISABLE
		CLR_BIT(SYSTICK->CTRL,TICKINT);
#else
	#error"Undefined SYSTICK iNTERRUPT State"
#endif
}
/***************************************************************************************************************************/
				/***** Enable Interrupt State in POST-Build Configuration*****/
/***************************************************************************************************************************/
void SYSTICK_InterruptEnable(void)//1
{
		SET_BIT(SYSTICK->CTRL,TICKINT);
}
/***************************************************************************************************************************/
				/***** Disable Interrupt State in POST-Build Configuration*****/
/***************************************************************************************************************************/
void SYSTICK_InterruptDisable(void)//2
{
		CLR_BIT(SYSTICK->CTRL,TICKINT);
}
/***************************************************************************************************************************/
				/*****To Start The SYSTICK Counter*****/
/***************************************************************************************************************************/

void SYSTICK_Start(u32 Load)//3
{
	SYSTICK->LOAD = Load;
	SYSTICK->VAL = 0;
	SYSTICK->CTRL |= 1;
}
/***************************************************************************************************************************/
				/*****A Function To Make Milli-Second Delay with Systick*****/
/***************************************************************************************************************************/
void SYSTICK_Delay_ms(u32 Value)//4
{
#if CLK_SOURCE == CLK_SOURCE_AHB_8
	SYSTICK_Start(Value*2000);
	while(!SYSTICK_ReadFlag());
#elif CLK_SOURCE == CLK_SOURCE_AHB
	SYSTICK_Start(Value*16000);
	while(!SYSTICK_ReadFlag());
#else
	#error"Undefined Clk Source"
#endif
}
/***************************************************************************************************************************/
				/*****A Function To Make Micro-Second Delay with Systick*****/
/***************************************************************************************************************************/
void SYSTICK_Delay_Us(u8 Value)//5
{
#if CLK_SOURCE == CLK_SOURCE_AHB_8
	SYSTICK_Start(Value*2);
	while(!SYSTICK_ReadFlag());
#elif CLK_SOURCE == CLK_SOURCE_AHB
	SYSTICK_Start(Value*16);
	while(!SYSTICK_ReadFlag());
#else
	#error"Undefined Clk Source"
#endif
}
/***************************************************************************************************************************/
				/*****A Function To Read The Counter Done Counting and Reached Zero*****/
/***************************************************************************************************************************/
u8 SYSTICK_ReadFlag(void)//6
{
	u8 Flag = 0;
	Flag = GET_BIT(SYSTICK->CTRL,COUNTFLAG);
	return Flag;
}
/***************************************************************************************************************************/
				/*****A Function To Disable Counter For POST-Build Configuration*****/
/***************************************************************************************************************************/
void SYSTICK_CounterDisable(void)//7
{
	CLR_BIT(SYSTICK->CTRL,ENABLE);
}
/***************************************************************************************************************************/
				/*****A Function To Enable Counter For POST-Build Configuration*****/
/***************************************************************************************************************************/
void SYSTICK_CounterEnable(void)//8
{
	SET_BIT(SYSTICK->CTRL,ENABLE);
}
/***************************************************************************************************************************/
				/*****Call Back Function To Handle Systick interrupts*****/
/***************************************************************************************************************************/
void SYSTICK_CallBackms(void(*ptr)(void),u32 Time_ms)//9
{
	SYSTICK_PTR = ptr;
	SYSTICK_Start(Time_ms*2000);
	SYSTICK_InterruptEnable();
}
void SYSTICK_CallBackus(void(*ptr)(void),u32 Time_us)//9
{
	SYSTICK_PTR = ptr;
	SYSTICK_Start(Time_us*2);
	SYSTICK_InterruptEnable();
}
/***************************************************************************************************************************/
				/*****A Function To Get The Counter Remaining Time*****/
/***************************************************************************************************************************/
u32 SYSTICK_RemainingTime(void)
{

	return  SYSTICK->VAL;
}
/***************************************************************************************************************************/
				/*****A Function To Get The Counter Elapsed Time*****/
/***************************************************************************************************************************/
u32 SYSTICK_ElapsedTime(void)
{
	return (SYSTICK->LOAD - SYSTICK->VAL);
}
/***************************************************************************************************************************/
											/*****SYSTICK HANDLER*****/
/***************************************************************************************************************************/
void SysTick_Handler (void)
{
	 if(SYSTICK_PTR!= NULL)
	{
	SYSTICK_PTR();
	SYSTICK_ReadFlag();
	 }
	 else
	 {
		 //donothing;
	 }
}





