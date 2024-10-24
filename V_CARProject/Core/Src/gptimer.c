/*
 * gptimer.c
 *
 *  Created on: Aug 15, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<GPTimer.h>
#include<GPTimers_Prv.h>
#include<GPTimer_Cfg.h>
#include<TIMER1_Prv.h>
void(*TIMER2_PTR)(void) = NULL;
void(*TIMER3_PTR)(void) = NULL;
void(*TIMER4_PTR)(void) = NULL;
void(*TIMER5_PTR)(void) = NULL;
void(*TIMER9_PTR)(void) = NULL;
void(*TIMER10_PTR)(void) = NULL;
void(*TIMER11_PTR)(void) = NULL;
void TIMER2_Start(void)
{
	/*Set the PreScaler For Timer 2*/
	TIMER2 -> PSC = (TIMER2_PRESCALER-1);
	/*Event Generation Update for Counter Reinitilization */
#if UpDateRequest_SRC_2T == UpDate2_On
	SET_BIT(TIMER2->CR1,CR1_URS);
#elif UpDateRequest_SRC_2T == UpDate2_Off
	SET_BIT(TIMER2->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer2"
#endif
#if UpDate_Generation_2T == Update2_Gen_On
	SET_BIT(TIMER2->EGR,EGR_UG);
#elif UpDate_Generation_2T == Update2_Gen_Off
	CLR_BIT(TIMER2->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer2"
#endif
#if ONE_PULSE_MODE_2T == One_Pulse2_On
	SET_BIT(TIMER2->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_2T == One_Pulse2_Off
	CLR_BIT(TIMER2->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer2"
#endif
#if COUNTER_STATE_2T == Counter2_Enable
	SET_BIT(TIMER2->CR1,CR1_CEN);
#elif COUNTER_STATE_2T == Counter2_Disable
	CLR_BIT(TIMER2->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer2"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_2T  == Auto2_RE_PRE_Enable
	SET_BIT(TIMER2->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_2T  == Auto2_RE_PRE_Disable
	CLR_BIT(TIMER2->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload Preload State For Timer2"
#endif

}
void TIMER2_MasterMode(TIMx_MM MM_Select,u16 Duration)
{
	TIMER2->ARR = (Duration-1);
	TIMER2->CR2 &=~ ((MMS_MUSK)<<CR2_MMS1);
	TIMER2->CR2 |= (MM_Select<<CR2_MMS1);
	TIMER2_Start();
	SET_BIT(TIMER2->CR1,CR1_CEN);
}
void TIMER3_Start(void)
{
	/*Set the PreScaler For Timer 3*/
	TIMER3 -> PSC = (TIMER3_PRESCALER-1);
	/*Event Generation Update for Counter Reinitilization */
#if UpDateRequest_SRC_3T == UpDate3_On
	SET_BIT(TIMER3->CR1,CR1_URS);
#elif UpDateRequest_SRC_3T == UpDate3_Off
	SET_BIT(TIMER3->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer3"
#endif
#if UpDate_Generation_3T == Update3_Gen_On
	SET_BIT(TIMER3->EGR,EGR_UG);
#elif UpDate_Generation_3T == Update3_Gen_Off
	CLR_BIT(TIMER3->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer3"
#endif
#if ONE_PULSE_MODE_3T == One_Pulse3_On
	SET_BIT(TIMER3->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_3T == One_Pulse3_Off
	CLR_BIT(TIMER3->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer3"
#endif
#if COUNTER_STATE_3T == Counter3_Enable
	SET_BIT(TIMER3->CR1,CR1_CEN);
#elif COUNTER_STATE_3T == Counter3_Disable
	CLR_BIT(TIMER3->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer3"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_3T  == Auto3_RE_PRE_Enable
	SET_BIT(TIMER3->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_3T  == Auto3_RE_PRE_Disable
	CLR_BIT(TIMER3->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload Preload State For Timer3 "
#endif
}
void TIMER4_Start(void)
{
	/*Set the PreScaler For Timer 3*/
	TIMER4 -> PSC = (TIMER4_PRESCALER-1);
	/*Event Generation Update for Counter Reinitilization */
#if UpDateRequest_SRC_4T == UpDate4_On
	SET_BIT(TIMER4->CR1,CR1_URS);
#elif UpDateRequest_SRC_4T == UpDate4_Off
	SET_BIT(TIMER4->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer4"
#endif
#if UpDate_Generation_4T == Update4_Gen_On
	SET_BIT(TIMER4->EGR,EGR_UG);
#elif UpDate_Generation_4T == Update4_Gen_Off
	CLR_BIT(TIMER4->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer4"
#endif
#if ONE_PULSE_MODE_4T == One_Pulse4_On
	SET_BIT(TIMER4->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_4T == One_Pulse4_Off
	CLR_BIT(TIMER4->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer4"
#endif
#if COUNTER_STATE_4T == Counter4_Enable
	SET_BIT(TIMER4->CR1,CR1_CEN);
#elif COUNTER_STATE_4T == Counter4_Disable
	CLR_BIT(TIMER4->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer4"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_4T  == Auto4_RE_PRE_Enable
	SET_BIT(TIMER4->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_4T  == Auto4_RE_PRE_Disable
	CLR_BIT(TIMER4->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload Preload State For Timer4 "
#endif
}
void TIMER5_Start(void)
{
	/*Set the PreScaler For Timer 2*/
	TIMER5 -> PSC = (TIMER5_PRESCALER-1);
	/*Event Generation Update for Counter Reinitilization */
#if UpDateRequest_SRC_5T == UpDate5_On
	SET_BIT(TIMER5->CR1,CR1_URS);
#elif UpDateRequest_SRC_5T == UpDate5_Off
	SET_BIT(TIMER5->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer2"
#endif
#if UpDate_Generation_5T == Update5_Gen_On
	SET_BIT(TIMER5->EGR,EGR_UG);
#elif UpDate_Generation_5T == Update5_Gen_Off
	CLR_BIT(TIMER5->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer2"
#endif
#if ONE_PULSE_MODE_5T == One_Pulse5_On
	SET_BIT(TIMER5->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_5T == One_Pulse5_Off
	CLR_BIT(TIMER5->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer2"
#endif
#if COUNTER_STATE_5T == Counter5_Enable
	SET_BIT(TIMER5->CR1,CR1_CEN);
#elif COUNTER_STATE_5T == Counter5_Disable
	CLR_BIT(TIMER5->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer5"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_5T  == Auto5_RE_PRE_Enable
	SET_BIT(TIMER5->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_5T  == Auto5_RE_PRE_Disable
	CLR_BIT(TIMER5->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload Preload State For Timer5"
#endif

}
void TIMER9_Start(void)
{
	/*Set the PreScaler For Timer 9*/
	TIMER9 -> PSC = (TIMER9_PRESCALER-1);
	/*Event Generation Update for Counter ReInitilization */
#if UpDateRequest_SRC_9T == UpDate9_On
	SET_BIT(TIMER9->CR1,CR1_URS);
#elif UpDateRequest_SRC_9T == UpDate9_Off
	SET_BIT(TIMER9->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer9"
#endif
#if UpDate_Generation_9T == Update9_Gen_On
	SET_BIT(TIMER9->EGR,EGR_UG);
#elif UpDate_Generation_9T == Update9_Gen_Off
	CLR_BIT(TIMER9->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer9"
#endif
#if ONE_PULSE_MODE_9T == One_Pulse9_On
	SET_BIT(TIMER9->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_9T == One_Pulse9_Off
	CLR_BIT(TIMER9->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer9"
#endif
#if COUNTER_STATE_9T == Counter9_Enable
	SET_BIT(TIMER9->CR1,CR1_CEN);
#elif COUNTER_STATE_9T == Counter9_Disable
	CLR_BIT(TIMER9->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer9"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_9T  == Auto9_RE_PRE_Enable
	SET_BIT(TIMER9->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_9T  == Auto9_RE_PRE_Disable
	CLR_BIT(TIMER9->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload PreLoad State For Timer9"
#endif

}
void TIMER10_Start(void)
{
	/*Set the PreScaler For Timer 10*/
	TIMER10 -> PSC = (TIMER10_PRESCALER-1);
	/*Event Generation Update for Counter ReInitilization */
#if UpDateRequest_SRC_10T == UpDate10_On
	SET_BIT(TIMER10->CR1,CR1_URS);
#elif UpDateRequest_SRC_10T == UpDate10_Off
	SET_BIT(TIMER10->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer10"
#endif
#if UpDate_Generation_10T == Update10_Gen_On
	SET_BIT(TIMER10->EGR,EGR_UG);
#elif UpDate_Generation_10T == Update10_Gen_Off
	CLR_BIT(TIMER10->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer10"
#endif
#if ONE_PULSE_MODE_10T == One_Pulse10_On
	SET_BIT(TIMER10->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_10T == One_Pulse10_Off
	CLR_BIT(TIMER10->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer10"
#endif
#if COUNTER_STATE_10T == Counter10_Enable
	SET_BIT(TIMER10->CR1,CR1_CEN);
#elif COUNTER_STATE_10T == Counter10_Disable
	CLR_BIT(TIMER10->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer10"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_10T  == Auto10_RE_PRE_Enable
	SET_BIT(TIMER10->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_10T  == Auto10_RE_PRE_Disable
	CLR_BIT(TIMER10->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload PreLoad State For Timer10"
#endif

}
void TIMER11_Start(void)
{
	/*Set the PreScaler For Timer 10*/
	TIMER11 -> PSC = (TIMER11_PRESCALER-1);
	/*Event Generation Update for Counter ReInitilization */
#if UpDateRequest_SRC_11T == UpDate11_On
	SET_BIT(TIMER11->CR1,CR1_URS);
#elif UpDateRequest_SRC_11T == UpDate11_Off
	SET_BIT(TIMER11->CR1,CR1_URS);
#else
	#error"Undefined Update Request Source For Timer11"
#endif
#if UpDate_Generation_11T == Update11_Gen_On
	SET_BIT(TIMER11->EGR,EGR_UG);
#elif UpDate_Generation_11T == Update11_Gen_Off
	CLR_BIT(TIMER11->EGR,EGR_UG);
#else
	#error"Undefined Update Generation Mode For Timer11"
#endif
#if ONE_PULSE_MODE_11T == One_Pulse11_On
	SET_BIT(TIMER11->CR1,CR1_OPM);
#elif ONE_PULSE_MODE_11T == One_Pulse11_Off
	CLR_BIT(TIMER11->CR1,CR1_OPM);
#else
	#error"Undefined One Pulse Mode For Timer11"
#endif
#if COUNTER_STATE_11T == Counter11_Enable
	SET_BIT(TIMER11->CR1,CR1_CEN);
#elif COUNTER_STATE_11T == Counter11_Disable
	CLR_BIT(TIMER11->CR1,CR1_CEN);
#else
	#error"Undefined Counter State For Timer11"
#endif
#if AUTO_RELOAD_PRELOAD_STATE_11T  == Auto11_RE_PRE_Enable
	SET_BIT(TIMER11->CR1,CR1_ARPE);
#elif AUTO_RELOAD_PRELOAD_STATE_11T  == Auto11_RE_PRE_Disable
	CLR_BIT(TIMER11->CR1,CR1_ARPE);
#else
	#error"Undefined Auto Reload PreLoad State For Timer11"
#endif

}
void TIMERx_BusyWait_Delay(GPTIMERS_no Timer,u16 Duration)
{	 switch(Timer)
	{
	case Timer2:
		TIMER2_Start();
		TIMER2->ARR = (Duration-1);
		while(TIMER2->CR1 & 1);
		break;
	case Timer3:
			TIMER3_Start();
			TIMER3->ARR = (Duration-1);
			while(TIMER3->CR1 & 1);
			break;
	case Timer4:
		TIMER4_Start();
		TIMER4->ARR = (Duration-1);
		while(TIMER4->CR1 & 1);
		break;
	case Timer5:
		TIMER5_Start();
		TIMER5->ARR = (Duration-1);
		while(TIMER5->CR1 & 1);
		break;
	case Timer9:
		TIMER9_Start();
		TIMER9->ARR = (Duration-1);
		while(TIMER9->CR1 & 1);
		break;
	case Timer10:
		TIMER10_Start();
		TIMER10->ARR = (Duration-1);
		while(TIMER10->CR1 & 1);
		break;
	case Timer11:
		TIMER11_Start();
		TIMER11->ARR = (Duration-1);
		while(TIMER11->CR1 & 1);
		break;
	}
}

void TIMERx_Interrupt_State(GPTIMERS_no Timer,TIMx_InterruptSt INT_St)
{
	switch(Timer)
	{
	case Timer2:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER2->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER2->DIER,DIER_UIE);
			break;
		}

	break;

	case Timer3:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER3->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER3->DIER,DIER_UIE);
			break;
		}

	break;

	case Timer4:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER4->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER4->DIER,DIER_UIE);
			break;
		}

	break;

	case Timer5:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER5->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER5->DIER,DIER_UIE);
			break;
		}
	break;
	case Timer9:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER9->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER9->DIER,DIER_UIE);
			break;
		}
	break;
	case Timer10:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER10->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER10->DIER,DIER_UIE);
			break;
		}
	break;
	case Timer11:
		switch(INT_St)
		{
		case TIMx_Interruptdis:
			CLR_BIT(TIMER11->DIER,DIER_UIE);
			break;
		case TIMx_Interrupten:
			SET_BIT(TIMER11->DIER,DIER_UIE);
			break;
			}
	break;
	}
}

void TIMER2_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value)
{
	TIMER2_Start();
	switch(channel)
	{
	case TIMx_channel1:
		SET_BIT(TIMER2->CCMR1,CCMR1_OC1PE);//r
		TIMER2 -> CCMR1 |= (C_Mode<<4);//r
		TIMER2 -> CCER |= (1<<(TIMx_channel1*4));//r
		TIMER2 -> CCR1 = Compare_u16Value;//r
		break;
	case TIMx_channel2:
		SET_BIT(TIMER2->CCMR1,CCMR1_OC2PE);//r
		TIMER2 ->CCMR1 |= (C_Mode<<12);//r
		TIMER2 -> CCER |= (1<<(TIMx_channel2*4));//r
		TIMER2 -> CCR2 = Compare_u16Value;//r
		break;
	case TIMx_channel3:
		SET_BIT(TIMER2->CCMR2,CCMR2_OC3PE);//r
		TIMER2 -> CCMR2 |= (C_Mode<<4);//r
		TIMER2 -> CCER |= (1<<(TIMx_channel3*4));//r
		TIMER2 -> CCR3 = Compare_u16Value;//r
		break;
	case TIMx_channel4:
		SET_BIT(TIMER2->CCMR2,CCMR2_OC4PE);//r
		TIMER2 -> CCMR2 |= (C_Mode<<12);//r
		TIMER2 -> CCER |= (1<<(TIMx_channel4*4));//r
		TIMER2 -> CCR4 = Compare_u16Value;//r
		break;
	}
	TIMER2 ->ARR = (Duration - 1);//r
	SET_BIT(TIMER1 -> BDTR,BDTR_MOE);
	SET_BIT(TIMER2->CR1,CR1_CEN);//r
}
void TIMER3_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value)
{
	TIMER3_Start();
	switch(channel)
	{
	case TIMx_channel1:
		SET_BIT(TIMER3->CCMR1,CCMR1_OC1PE);//r
		TIMER3 -> CCMR1 |= (C_Mode<<4);//r
		TIMER3 -> CCER |= (1<<(TIMx_channel1*4));//r
		TIMER3 -> CCR1 = Compare_u16Value;//r
		break;
	case TIMx_channel2:
		SET_BIT(TIMER3->CCMR1,CCMR1_OC2PE);//r
		TIMER3 ->CCMR1 |= (C_Mode<<12);//r
		TIMER3 -> CCER |= (1<<(TIMx_channel2*4));//r
		TIMER3 -> CCR2 = Compare_u16Value;//r
		break;
	case TIMx_channel3:
		SET_BIT(TIMER3->CCMR2,CCMR2_OC3PE);//r
		TIMER3 -> CCMR2 |= (C_Mode<<4);//r
		TIMER3 -> CCER |= (1<<(TIMx_channel3*4));//r
		TIMER3 -> CCR3 = Compare_u16Value;//r
		break;
	case TIMx_channel4:
		SET_BIT(TIMER3->CCMR2,CCMR2_OC4PE);//r
		TIMER3 -> CCMR2 |= (C_Mode<<12);//r
		TIMER3 -> CCER |= (1<<(TIMx_channel4*4));//r
		TIMER3 -> CCR4 = Compare_u16Value;//r
		break;
	}
	TIMER3 ->ARR = (Duration - 1);//r
	SET_BIT(TIMER1 -> BDTR,BDTR_MOE);
	SET_BIT(TIMER3->CR1,CR1_CEN);//r
}
void TIMER4_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value)
{
	TIMER4_Start();
	switch(channel)
	{
	case TIMx_channel1:
		SET_BIT(TIMER4->CCMR1,CCMR1_OC1PE);//r
		TIMER4 -> CCMR1 |= (C_Mode<<4);//r
		TIMER4 -> CCER |= (1<<(TIMx_channel1*4));//r
		TIMER4 -> CCR1 = Compare_u16Value;//r
		break;
	case TIMx_channel2:
		SET_BIT(TIMER4->CCMR1,CCMR1_OC2PE);//r
		TIMER4 ->CCMR1 |= (C_Mode<<12);//r
		TIMER4 -> CCER |= (1<<(TIMx_channel2*4));//r
		TIMER4 -> CCR2 = Compare_u16Value;//r
		break;
	case TIMx_channel3:
		SET_BIT(TIMER4->CCMR2,CCMR2_OC3PE);//r
		TIMER4 -> CCMR2 |= (C_Mode<<4);//r
		TIMER4 -> CCER |= (1<<(TIMx_channel3*4));//r
		TIMER4 -> CCR3 = Compare_u16Value;//r
		break;
	case TIMx_channel4:
		SET_BIT(TIMER4->CCMR2,CCMR2_OC4PE);//r
		TIMER4 -> CCMR2 |= (C_Mode<<12);//r
		TIMER4 -> CCER |= (1<<(TIMx_channel4*4));//r
		TIMER4 -> CCR4 = Compare_u16Value;//r
		break;
	}
	TIMER4 ->ARR = (Duration - 1);//r
	SET_BIT(TIMER1 -> BDTR,BDTR_MOE);
	SET_BIT(TIMER4->CR1,CR1_CEN);//r
}
void TIMER5_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value)
{
	TIMER5_Start();
	switch(channel)
	{
	case TIMx_channel1:
		SET_BIT(TIMER5->CCMR1,CCMR1_OC1PE);//r
		TIMER5 -> CCMR1 |= (C_Mode<<4);//r
		TIMER5 -> CCER |= (1<<(TIMx_channel1*4));//r
		TIMER5 -> CCR1 = Compare_u16Value;//r
		break;
	case TIMx_channel2:
		SET_BIT(TIMER5->CCMR1,CCMR1_OC2PE);//r
		TIMER5 ->CCMR1 |= (C_Mode<<12);//r
		TIMER5 -> CCER |= (1<<(TIMx_channel2*4));//r
		TIMER5 -> CCR2 = Compare_u16Value;//r
		break;
	case TIMx_channel3:
		SET_BIT(TIMER5->CCMR2,CCMR2_OC3PE);//r
		TIMER5 -> CCMR2 |= (C_Mode<<4);//r
		TIMER5 -> CCER |= (1<<(TIMx_channel3*4));//r
		TIMER5 -> CCR3 = Compare_u16Value;//r
		break;
	case TIMx_channel4:
		SET_BIT(TIMER5->CCMR2,CCMR2_OC4PE);//r
		TIMER5 -> CCMR2 |= (C_Mode<<12);//r
		TIMER5 -> CCER |= (1<<(TIMx_channel4*4));//r
		TIMER5 -> CCR4 = Compare_u16Value;//r
		break;
	}
	TIMER5 ->ARR = (Duration - 1);//r
	SET_BIT(TIMER1 -> BDTR,BDTR_MOE);
	SET_BIT(TIMER5->CR1,CR1_CEN);//r
}
void TIMER2_CallBack(void(*PTR),u16 Duration)
{
	TIMER2_PTR = PTR ;
	TIMER2_Start();
	TIMER2->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer2,TIMx_Interrupten);
}
void TIMER3_CallBack(void(*PTR),u16 Duration)
{
	TIMER3_PTR = PTR ;
	TIMER3_Start();
	TIMER3->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer3,TIMx_Interrupten);
}
void TIMER4_CallBack(void(*PTR),u16 Duration)
{
	TIMER4_PTR = PTR ;
	TIMER4_Start();
	TIMER4->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer4,TIMx_Interrupten);
}
void TIMER5_CallBack(void(*PTR),u16 Duration)
{
	TIMER5_PTR = PTR ;
	TIMER5_Start();
	TIMER5->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer5,TIMx_Interrupten);
}
void TIMER9_CallBack(void(*PTR),u16 Duration)//Done
{
	TIMER9_PTR = PTR ;
	TIMER9_Start();
	TIMER9->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer9,TIMx_Interrupten);
}
void TIMER10_CallBack(void(*PTR),u16 Duration)
{
	TIMER10_PTR = PTR ;
	TIMER10_Start();
	TIMER10->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer10,TIMx_Interrupten);
}
void TIMER11_CallBack(void(*PTR),u16 Duration)
{
	TIMER11_PTR = PTR ;
	TIMER11_Start();
	TIMER11->ARR = (Duration-1);
	TIMERx_Interrupt_State(Timer11,TIMx_Interrupten);
}
void TIM2_IRQHandler(void)
{
	TIMER2_PTR();
	CLR_BIT(TIMER2->SR,SR_UIF);
}
void TIM3_IRQHandler(void)
{
	TIMER3_PTR();
	CLR_BIT(TIMER3->SR,SR_UIF);
}
void TIM4_IRQHandler(void)
{
	TIMER4_PTR();
	CLR_BIT(TIMER4->SR,SR_UIF);
}
void TIM5_IRQHandler(void)
{
	TIMER5_PTR();
	CLR_BIT(TIMER5->SR,SR_UIF);
}
void TIM1_BRK_TIM9_IRQHandler(void)
{
	TIMER9_PTR();
	CLR_BIT(TIMER9->SR,SR_UIF);
}
/*
void TIM1_UP_TIM10_IRQHandler(void)
{
	TIMER10_PTR();
	CLR_BIT(TIMER10->SR,SR_UIF);
}
*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	TIMER11_PTR();
	CLR_BIT(TIMER11->SR,SR_UIF);
}
