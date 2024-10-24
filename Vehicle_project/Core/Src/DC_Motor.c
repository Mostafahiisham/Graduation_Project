/*
 * DC_Motor.c
 *
 *  Created on: Sep 2, 2023
 *      Author: Mostafa Hisham
 *      Layer : HAL
 *      SWC   : DC- MOTOR - IMPLEMENTATION
 */
#include<Std_Types.h>
#include<Macros.h>
#include<MGPIO.h>
#include<DC_Motor.h>
#include<DC_Motor_cfg.h>

/*H-Bridge Connection */

u16 motor_speed;
static void DC_Motor_Init(void);
static void DC_Motor_Init(void)
{
	for(u8 i = 0;i<8;i++)
	{
	GPIO_SetMode(MOTOR_PORT,i,OUTPUT);
	GPIO_SetOutputMode(MOTOR_PORT, i, PUSH_PULL);
	GPIO_SetOutputSpeed(MOTOR_PORT, i, MEDIUM_SPEED);
	}

}
void DCMOTOR_RotateCounterClockWise(Motor_Number motor)
{
	DC_Motor_Init();

	switch (motor)
	{
	case 0:
	GPIO_SetPinValue(MOTOR_PORT,MOTOR1_PIN1,PIN_HIGH);
	GPIO_SetPinValue(MOTOR_PORT,MOTOR1_PIN2, PIN_LOW);
	break;
	case 1:
	GPIO_SetPinValue(MOTOR_PORT,MOTOR2_PIN1,PIN_HIGH);
	GPIO_SetPinValue(MOTOR_PORT,MOTOR2_PIN2, PIN_LOW);
	break;
	case 2:
	GPIO_SetPinValue(MOTOR_PORT,MOTOR3_PIN1,PIN_HIGH);
	GPIO_SetPinValue(MOTOR_PORT,MOTOR3_PIN2, PIN_LOW);

	break;
	case 3:
	GPIO_SetPinValue(MOTOR_PORT,MOTOR4_PIN1,PIN_HIGH);
	GPIO_SetPinValue(MOTOR_PORT,MOTOR4_PIN2, PIN_LOW);
	break;
	default:
		break;
	}
}
void DCMOTOR_RotateClockwise(Motor_Number motor)
{
	DC_Motor_Init();
	switch (motor)
	{
	case 0:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR1_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR1_PIN2, PIN_HIGH);
	break;
	case 1:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR2_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR2_PIN2, PIN_HIGH);
	break;
	case 2:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR3_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR3_PIN2, PIN_HIGH);
	break;
	case 3:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR4_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR4_PIN2, PIN_HIGH);
	break;
	default:
		break;
	}
}
void DCMOTOR_Stop(Motor_Number motor)
{
	DC_Motor_Init();
	switch(motor)
	{
	case 0:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR1_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR1_PIN2,PIN_LOW);
	break;
	case 1:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR2_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR2_PIN2,PIN_LOW);
	break;
	case 2:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR3_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR3_PIN2,PIN_LOW);
	break;
	case 3:
		GPIO_SetPinValue(MOTOR_PORT,MOTOR4_PIN1,PIN_LOW);
		GPIO_SetPinValue(MOTOR_PORT,MOTOR4_PIN2,PIN_LOW);
	break;
	default:
		break;
	}
}
