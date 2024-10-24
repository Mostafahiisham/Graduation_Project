/*
 * MGPIO.h
 *
 *  Created on: Jul 20, 2024
 *      Author: Mostafa Hisham
 */

#ifndef MGPIO_H_
#define MGPIO_H_

typedef enum{
	INPUT,				//INPUT MODE
	OUTPUT,				//OUTPUT MODE
	ALTERNATE,			//ALTERNATE FUNCTION MODE
	ANALOG				//ANALOG MODE
}Mode_t;

typedef enum{
	PIN0,
	PIN1,
	PIN2,
	PIN3,
	PIN4,
	PIN5,
	PIN6,
	PIN7,
	PIN8,
	PIN9,
	PIN10,
	PIN11,
	PIN12,
	PIN13,
	PIN14,
	PIN15
}PIN_Number;
typedef enum{
	PORTA,
	PORTB,
	PORTC
}PORT_t;

typedef enum{
	PUSH_PULL,
	OPEN_DRAIN
}OUTPUT_t;

typedef enum{
	LOW_SPEED,
	MEDIUM_SPEED,
	HIGH_SPEED,
	VERY_HIGH_SPEED
}OUTPUTSpeed_t;

typedef enum{
	NOPUSH_NOPULL,
	PULL_UP,
	PULL_DOWN
}PULL_t;

typedef enum{
	PIN_LOW,
	PIN_HIGH
}PIN_Value;
typedef enum{
	AF0,
	AF1,
	AF2,
	AF3,
	AF4,
	AF5,
	AF6,
	AF7,
	AF8,
	AF9,
	AF10,
	AF11,
	AF12,
	AF13,
	AF14,
	AF15
}AlterFunc_Number;
typedef enum{
	PORT_LOW,
	PORT_FIRST_HALF_HIGH = 0XFF,
	PORT_LAST_HALF_HIGH = 0xFF00,
	PORT_HIGH = 0XFFFF
}PORT_Value;

typedef enum{
	PORT_INPUT,
	PORT_FIRST_HALF_OUTPUT = 0x5555,
	PORT_LAST_HALF_OUTPUT = 0x55550000,
	PORT_OUTPUT = 0x55555555
}PORT_Mode;

void GPIO_SetMode(PORT_t Port,PIN_Number Pin,Mode_t Mode);

void GPIO_SetOutputMode(PORT_t Port,PIN_Number Pin,OUTPUT_t Omode);

void GPIO_SetOutputSpeed(PORT_t Port,PIN_Number Pin,OUTPUTSpeed_t O_Speed);

void GPIO_SetPinValue(PORT_t Port,PIN_Number Pin,PIN_Value P_Value);

void GPIO_SetPullType(PORT_t Port,PIN_Number Pin,PULL_t PULL_Type);

void GPIO_GetPinValue(PORT_t Port,PIN_Number Pin,u32 *copy_PinValue);

void GPIO_SetAtomicPinValue(PORT_t Port,PIN_Number Pin,PIN_Value P_Value);

void GPIO_TogglePinValue(PORT_t Port,PIN_Number Pin);

void GPIO_SetPortMode(PORT_t Port,PORT_Mode P_Mode);//NOT WORK

void GPIO_SetPortValue(PORT_t Port,PORT_Value P_Value);

void GPIO_SetAlternateFunc(PORT_t Port,PIN_Number Pin,AlterFunc_Number AF_Number);




#endif /* MGPIO_H_ */
