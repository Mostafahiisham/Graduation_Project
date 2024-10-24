/*
 * TIMER1.h
 *
 *  Created on: Aug 13, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_TIMER1_H_
#define INC_TIMER1_H_

#define UpDate_On			1
#define UpDate_Off			2

#define Update_Gen_On 		1
#define Update_Gen_Off		2

#define One_Pulse_On		1
#define One_Pulse_Off		2

#define  Counter_Enable		1
#define  Counter_Disable	2

#define Auto_RE_PRE_Enable	1
#define Auto_RE_PRE_Disable 2

#define Micro_Prescaler     16
#define Milli_Prescaler		16000
typedef enum{
	TIM1_Interruptdis,
	TIM1_Interrupten
}TIM1_InterruptSt;

typedef enum{
	channel1,
	channel2,
	channel3,
	channel4
}Channel_no;
typedef enum{
	Frozen,
	ACTIVE_ON_MATCH,
	INACTIVE_ON_MATCH,
	TOGGLE,
	FORCED_INACTIVE,
	FORCED_ACTIVE,
	PWM_MODE1,
	PWM_MODE2
}Compare_t;
typedef enum{
	TIM1_Capture_Rising,
	TIM1_Capture_FALLING
}TIM1_Capture;
typedef enum{
	Channel_output,
	channel_inputMap_TI1,
	channel_inputMap_TI2,
	channel_inputMap_TRC
}Channel_Dir;

void TIMER1_Start(void);
void TIMER1_BusyWait_Delay(u16 Duration);
void TIMER1_Interrupt_State(TIM1_InterruptSt INT_St);
void TIMER1_CallBack(void(*PTR)(void),u16 Duration);
void TIMER1_SetOutputCompareMode(Channel_no channel,Compare_t C_Mode,u16 Duration,u16 Compare_u16Value);
void TIMER1_SetInputCaptureUnit(Channel_no channel,Channel_Dir Dir,u16 Duration,TIM1_Capture Edge);
u32 TIMER1_GetCapture(Channel_no channel);
#endif /* INC_TIMER1_H_ */
