/*
 * SYSTICK.h
 *
 *  Created on: Aug 2, 2024
 *      Author: Mostafa Hisham
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#define	CLK_SOURCE_AHB_8   		1
#define CLK_SOURCE_AHB	   		2

#define SYSTICK_INT_ENABLE 		1
#define SYSTICK_INT_DISABLE 	2


void SYSTICK_Init(void);//1
void SYSTICK_InterruptEnable(void);//2
void SYSTICK_InterruptDisable(void);//11
void SYSTICK_Start(u32 Load);//5
void SYSTICK_Delay_ms(u32 Value);//3
void SYSTICK_Delay_Us(u8 Value);//4
u32 SYSTICK_RemainingTime(void);
u32 SYSTICK_ElapsedTime(void);
u8 SYSTICK_ReadFlag(void);//6
void SYSTICK_CounterDisable(void);//7
void SYSTICK_CounterEnable(void);//8
void SYSTICK_CallBackms(void(*ptr)(void),u32 Time_ms);//9
void SYSTICK_CallBackus(void(*ptr)(void),u32 Time_us);



#endif /* SYSTICK_H_ */
