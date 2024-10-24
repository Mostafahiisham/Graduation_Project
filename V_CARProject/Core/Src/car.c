/*
 * car.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<MGPIO.h>
#include<STP.h>
#include<TIMER1.h>
#include<car.h>

void CAR_Movement_Init(void)
{
	STP_Init();
	/*Enable Motors Pin*/
	GPIO_SetMode(PORTA, PIN8, ALTERNATE);
	GPIO_SetMode(PORTA, PIN9, ALTERNATE);
	GPIO_SetAlternateFunc(PORTA, PIN8, AF1);
	GPIO_SetAlternateFunc(PORTA, PIN9, AF1);
}


void CAR_Moveforward(void)
{
	STP_SendData();
	STP_ShiftData(MOVE_FORWARD);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
void CAR_MovebackWard(void)
{
	STP_SendData();
	STP_ShiftData(MOVE_BACKWARD);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
void CAR_Moveright(void)
{
	STP_SendData();
	STP_ShiftData(MOVE_RIGHT);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
void CAR_Moveleft(void)
{

	STP_SendData();
	STP_ShiftData(MOVE_LEFT);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}

void CAR_sTOP(void)
{
	STP_SendData();
	STP_ShiftData(STOP_CAR);
	STP_SendData();
}
void CAR_firstSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED1);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED1);
}
void CAR_secondSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED2);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED2);
}
void CAR_thirdSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED3);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED3);
}

void CAR_fourthSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED4);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED4);
}
void CAR_fifthSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
