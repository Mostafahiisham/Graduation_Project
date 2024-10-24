/*
 * car.c
 *
 *  Created on: Oct 24, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<STP.h>
#include<TIMER1.h>
#include<car.h>

void CAR_MovementInit(void)
{
	STP_Init();
}

void CAR_MoveForward(void)
{
	STP_SendData();
	STP_ShiftData(MOVE_FORWARD);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
void CAR_MoveBackWard(void)
{
	STP_SendData();
	STP_ShiftData(MOVE_BACKWARD);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
void CAR_MoveRight(void)
{
	STP_SendData();
	STP_ShiftData(MOVE_RIGHT);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
void CAR_MoveLeft(void)
{

	STP_SendData();
	STP_ShiftData(MOVE_LEFT);
	STP_SendData();
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}

void CAR_STOP(void)
{
	STP_SendData();
	STP_ShiftData(STOP_CAR);
	STP_SendData();
}
void CAR_FirstSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED1);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED1);
}
void CAR_SecondSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED2);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED2);
}
void CAR_ThirdSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED3);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED3);
}

void CAR_FourthSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED4);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED4);
}
void CAR_FifthSpeed(void)
{
	TIMER1_SetOutputCompareMode(channel1, PWM_MODE1, 65500, SPEED5);
	TIMER1_SetOutputCompareMode(channel2, PWM_MODE1, 65500, SPEED5);
}
