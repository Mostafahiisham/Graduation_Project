/*
 * Timer1.c
 *
 *  Created on: Aug 13, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<TIMER1.h>
#include<TIMER1_Prv.h>
#include<TIMER1_Cfg.h>
static void (*TIM1_PTR)(void) = NULL;
void TIMER1_Start(void)
{
	/*Set the PreScaler For Timer 1*/
	TIMER1 -> PSC = (TIMER1_PRESCALER-1);
	/*Event Generation Update for Counter Reinitilization */
#if UpDateRequest_SRC == UpDate_On
	SET_BIT(TIMER1->CR1,CR1_URS);
#elif UpDateRequest_SRC == UpDate_Off
	SET_BIT(TIMER1->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source"
#endif
#if UpDate_Generation == Update_Gen_On
	SET_BIT(TIMER1->EGR,EGR_UG);
#elif UpDate_Generation == Update_Gen_Off
	CLR_BIT(TIMER1->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode"
#endif
#if ONE_PULSE_MODE == One_Pulse_On
	SET_BIT(TIMER1->CR1,CR1_OPM);
#elif ONE_PULSE_MODE == One_Pulse_Off
	CLR_BIT(TIMER1->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode"
#endif
#if COUNTER_STATE == Counter_Enable
	SET_BIT(TIMER1->CR1,CR1_CEN);
#elif COUNTER_STATE == Counter_Disable
	CLR_BIT(TIMER1->CR1,CR1_CEN);
#else
	#error"Undefined Counter State"
#endif
#if AUTO_RELOAD_PRELOAD_STATE  == Auto_RE_PRE_Enable
	SET_BIT(TIMER1->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE  == Auto_RE_PRE_Disable
	CLR_BIT(TIMER1->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload Preload State"
#endif

}
void TIMER1_BusyWait_Delay(u16 Duration)
{
	TIMER1_Start();
	TIMER1->ARR = (Duration-1);
	while(TIMER1->CR1 & 1);
}
void TIMER1_Interrupt_State(TIM1_InterruptSt INT_St)
{
	switch(INT_St)
	{
	case TIM1_Interruptdis:
		CLR_BIT(TIMER1->DIER,DIER_UIE);
		break;
	case TIM1_Interrupten:
		SET_BIT(TIMER1->DIER,DIER_UIE);
		break;
	}
}

void TIMER1_CallBack(void(*PTR)(void),u16 Duration)
{	TIM1_PTR = PTR;
	TIMER1_Start();
	TIMER1->ARR = (Duration-1);
	TIMER1_Interrupt_State(TIM1_Interrupten);
}

void TIMER1_SetOutputCompareMode(Channel_no channel,Compare_t C_Mode,u16 Duration,u16 Compare_u16Value)
{
	TIMER1_Start();
	switch(channel)
	{
	case channel1:
		SET_BIT(TIMER1->CCMR1,CCMR1_OC1PE);
		TIMER1 -> CCMR1 |= (C_Mode<<4);
		TIMER1 -> CCER |= (1<<(channel1*4));
		TIMER1 -> CCR1 = Compare_u16Value;
		break;
	case channel2:
		SET_BIT(TIMER1->CCMR1,CCMR1_OC2PE);
		TIMER1 ->CCMR1 |= (C_Mode<<12);
		TIMER1 -> CCER |= (1<<(channel2*4));
		TIMER1 -> CCR2 = Compare_u16Value;
		break;
	case channel3:
		SET_BIT(TIMER1->CCMR2,CCMR2_OC3PE);
		TIMER1 -> CCMR2 |= (C_Mode<<4);
		TIMER1 -> CCER |= (1<<(channel3*4));
		TIMER1 -> CCR3 = Compare_u16Value;
		break;
	case channel4:
		SET_BIT(TIMER1->CCMR2,CCMR2_OC4PE);
		TIMER1 -> CCMR2 |= (C_Mode<<12);
		TIMER1 -> CCER |= (1<<(channel4*4));
		TIMER1 -> CCR4 = Compare_u16Value;
		break;
	}
	TIMER1 ->ARR = (Duration - 1);
	SET_BIT(TIMER1->BDTR,BDTR_MOE);
	SET_BIT(TIMER1->CR1,CR1_CEN);
}
void TIMER1_SetInputCaptureUnit(Channel_no channel,Channel_Dir Dir,u16 Duration,TIM1_Capture Edge)
{
	/*Initialize the Timer*/
	TIMER1_Start();
	/*SET The Duration For The Auto Reload Register*/
	TIMER1 -> ARR = (Duration - 1);
	/*Channel Selection */
	switch(channel)
	{
	case channel1:
		/*Set the channel 1 as input capture AND Map it to the timer*/
		TIMER1 -> CCMR1 |= (Dir);
		/*Select the Edge for capture operation for channel 1*/
		TIMER1 -> CCER |=(Edge<<CCER_CC1P);
		TIMER1 -> CCER |=(Edge<<CCER_CC1NP);
		/*Capture/Compare Channel Enable for channel 1*/
		TIMER1 -> CCER |= (1<<CCER_CC1E);

		break;
	case channel2:
		/*Set the channel 2 as input capture*/
		TIMER1 -> CCMR1 |= (Dir);
		/*Select the Edge for Capture Operation for channel 2*/
		TIMER1 -> CCER |=(Edge<<CCER_CC2P);
		/*Capture/Compare channel Enable for channel 2*/
		TIMER1 ->CCER |=(1<<CCER_CC2E);

		break;
	case channel3:
		/*Set the channel 3 as input capture*/
		TIMER1 -> CCMR2 |= (Dir);
		/*Select the Edge for Capture Operation for channel 3*/
		TIMER1 -> CCER |=(Edge<<CCER_CC3P);
		/*Capture/Compare channel Enable for channel 3*/
		TIMER1 ->CCER |= (1<<CCER_CC3E);
		break;
	case channel4:
		/*Set the channel 4 as input capture*/
		TIMER1 -> CCMR2 |= (Dir);
		/*Select the Edge for Capture Operation for channel 4*/
		TIMER1 -> CCER |=(Edge<<CCER_CC4P);
		/*Capture/Compare channel Enable for channel 4*/
		TIMER1 ->CCER |= (1<<CCER_CC4E);
		break;
	default:
		break;
		}
	/*Counter Enable*/
	SET_BIT(TIMER1->CR1,CR1_CEN);
}
u32 TIMER1_GetCapture(Channel_no channel)
{
	u32 Copy_u32Capture;
	u32 Frequency = 0;
	switch(channel)
	{
	case channel1:
	Copy_u32Capture = TIMER1 ->CCR1;
		break;
	case channel2:
	Copy_u32Capture = TIMER1 ->CCR2;
		break;
	case channel3:
	Copy_u32Capture = TIMER1 ->CCR3;
		break;
	case channel4:
	Copy_u32Capture = TIMER1 ->CCR4;
		break;
	default:
		break;

	}
	if(Copy_u32Capture != 0)
	{
	Frequency = 100000 / Copy_u32Capture;
	}
	else
	{
		Frequency = 0;
	}
	return  Frequency;
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	TIM1_PTR();
	CLR_BIT(TIMER1->SR,SR_UIF);
}

