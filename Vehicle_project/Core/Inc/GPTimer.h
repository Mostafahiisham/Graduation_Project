/*
 * GPTimer.h
 *
 *  Created on: Aug 15, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_GPTIMER_H_
#define INC_GPTIMER_H_

#define UpDate2_On					1
#define	UpDate2_Off					2

#define Update2_Gen_On				1
#define Update2_Gen_Off				2


#define One_Pulse2_On				1
#define	One_Pulse2_Off				2

#define Counter2_Enable				1
#define Counter2_Disable			2

#define Auto2_RE_PRE_Enable 		1
#define Auto2_RE_PRE_Disable		2
/**** Data Types for Timer3****/
#define UpDate3_On					1
#define	UpDate3_Off					2

#define Update3_Gen_On				1
#define Update3_Gen_Off				2

#define One_Pulse3_On				1
#define	One_Pulse3_Off				2

#define Counter3_Enable				1
#define Counter3_Disable			2

#define Auto3_RE_PRE_Enable 		1
#define Auto3_RE_PRE_Disable		2
/**********Timer4***********/
#define UpDate4_On					1
#define	UpDate4_Off					2

#define Update4_Gen_On				1
#define Update4_Gen_Off				2

#define One_Pulse4_On				1
#define	One_Pulse4_Off				2

#define Counter4_Enable				1
#define Counter4_Disable			2

#define Auto4_RE_PRE_Enable 		1
#define Auto4_RE_PRE_Disable		2
/*******************/

#define UpDate5_On					1
#define	UpDate5_Off					2

#define Update5_Gen_On				1
#define Update5_Gen_Off				2


#define One_Pulse5_On				1
#define	One_Pulse5_Off				2

#define Counter5_Enable				1
#define Counter5_Disable			2

#define Auto5_RE_PRE_Enable 		1
#define Auto5_RE_PRE_Disable		2
/***************************************/

#define UpDate9_On					1
#define	UpDate9_Off					2

#define Update9_Gen_On				1
#define Update9_Gen_Off				2


#define One_Pulse9_On				1
#define	One_Pulse9_Off				2

#define Counter9_Enable				1
#define Counter9_Disable			2

#define Auto9_RE_PRE_Enable 		1
#define Auto9_RE_PRE_Disable		2
/***************************************/

#define UpDate10_On					1
#define	UpDate10_Off					2

#define Update10_Gen_On				1
#define Update10_Gen_Off				2


#define One_Pulse10_On				1
#define	One_Pulse10_Off				2

#define Counter10_Enable				1
#define Counter10_Disable			2

#define Auto10_RE_PRE_Enable 		1
#define Auto10_RE_PRE_Disable		2

/***************************************/

#define UpDate11_On					1
#define	UpDate11_Off					2

#define Update11_Gen_On				1
#define Update11_Gen_Off				2


#define One_Pulse11_On				1
#define	One_Pulse11_Off				2

#define Counter11_Enable				1
#define Counter11_Disable			2

#define Auto11_RE_PRE_Enable 		1
#define Auto11_RE_PRE_Disable		2
typedef enum{
	Timer2,
	Timer3,
	Timer4,
	Timer5,
	Timer9,
	Timer10,
	Timer11
}GPTIMERS_no;
typedef enum{
	TIMx_Interruptdis,
	TIMx_Interrupten
}TIMx_InterruptSt;

typedef enum{
	TIMx_Frozen,
	TIMx_ACTIVE_ON_MATCH,
	TIMx_INACTIVE_ON_MATCH,
	TIMx_TOGGLE,
	TIMx_FORCED_INACTIVE,
	TIMx_FORCED_ACTIVE,
	TIMx_PWM_MODE1,
	TIMx_PWM_MODE2
}TIMx_Compare_t;
typedef enum{
	TIMx_channel1,
	TIMx_channel2,
	TIMx_channel3,
	TIMx_channel4
}TIMx_Channel_no;
typedef enum{
	TIMx_Reset,
	TIMx_Enable,
	TIMx_Update,
	TIMx_OC1TRGO,
	TIMx_OC2TRGO,
	TIMx_OC3TRGO,
	TIMx_OC4TRGO
}TIMx_MM;

void TIMER2_Start(void);
void TIMER3_Start(void);
void TIMER4_Start(void);
void TIMER5_Start(void);
void TIMER9_Start(void);
void TIMER10_Start(void);
void TIMER11_Start(void);
void TIMER2_MasterMode(TIMx_MM MM_Select,u16 Duration);
void TIMERx_BusyWait_Delay(GPTIMERS_no Timer,u16 Duration);
void TIMERx_Interrupt_State(GPTIMERS_no Timer,TIMx_InterruptSt INT_St);
void TIMER2_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value);
void TIMER3_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value);
void TIMER4_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value);
void TIMER5_SetOutputCompareMode(TIMx_Channel_no channel,TIMx_Compare_t C_Mode,u16 Duration,u16 Compare_u16Value);
void TIMER2_CallBack(void(*PTR),u16 Duration);
void TIMER3_CallBack(void(*PTR),u16 Duration);
void TIMER4_CallBack(void(*PTR),u16 Duration);
void TIMER5_CallBack(void(*PTR),u16 Duration);
void TIMER9_CallBack(void(*PTR),u16 Duration);
void TIMER10_CallBack(void(*PTR),u16 Duration);
void TIMER11_CallBack(void(*PTR),u16 Duration);
#endif /* INC_GPTIMER_H_ */
