/*
 * main.c
 *
 *  Created on: Oct 18, 2024
 *      Author: Mostafa Hisham
 */
#include<Std_Types.h>
#include<Macros.h>
#include<MRCC.h>
#include<NVIC.h>
#include<MGPIO.h>
#include<USART.h>
#include<ADC.h>
#include<Volt_Sensor.h>
#include<car.h>

u8 Data = 0;
u16 ADC_Reading = 0;

void main(void)
{
	RCC_VInit();
	RCC_VEnableClkPeripheral(AHB1ENR, AHB1_GPIOA_EN);
	RCC_VEnableClkPeripheral(AHB1ENR, AHB1_GPIOB_EN);
	RCC_VEnableClkPeripheral(APB2ENR, APB2_USART1_EN);
	RCC_VEnableClkPeripheral(APB2ENR, APB2_TIM1_EN);
	RCC_VEnableClkPeripheral(APB2ENR, APB2_USART6_EN);
	RCC_VEnableClkPeripheral(APB2ENR, APB2_ADC1_EN);

	GPIO_SetMode(PORTB, PIN7, ALTERNATE);
	GPIO_SetAlternateFunc(PORTB, PIN7, AF7);
	USART_Init();
	NVIC_EnableInterrupt(USART1);
	USART1_RX_InterruptState(RX_INT_Enable);
	USART1_CallBack(CAR_Control, RX_Interrupt);

	/*********/
	ADC1_vInit(ADCx_Res12, PCLK2_2);
	NVIC_EnableInterrupt(ADC);
	GPIO_SetMode(PORTA, PIN5, ANALOG);
	ADC1_CallBack(Voltage_Read,Injected);
	ADC1_InterruptEnable(Injected);
	/****************/
	GPIO_SetMode(PORTA, PIN11, ALTERNATE);
	GPIO_SetMode(PORTA, PIN12, ALTERNATE);
	GPIO_SetAlternateFunc(PORTA, PIN11, AF8);
	GPIO_SetAlternateFunc(PORTA, PIN12, AF8);
	USART6_Init();
	NVIC_EnableInterrupt(USART6);
	USART6_TX_InterruptState(TX_INT_Enable);

	while(1)
	{
		Data = USART1_ReadDataRegister();
		ADC_Reading = ADC1_vReadInjectedSingleChannelAsynchronous(ADCx_CHANNEL5);
		ADC1_InjectedSwStartEnable();

	}

}
void CAR_Control(void)
{	u8 car_mode = Data;
	switch(car_mode)
	{
	/* In This case the car moves forward */
	case 'w':

		CAR_MoveForward();
	break;
	/* In This case the car moves Backward */
	case 's':
					//3a-2a-1a-1b-2b-4a-3b-4b
		CAR_MoveBackWard();
	break;
	/*In This case the car moves right*/
	case 'd':

		CAR_MoveRight();
	break;
	/*In This case the car moves left*/
	case'a':

		CAR_MoveLeft();
	break;
	case'z':

		CAR_STOP();
	break;
	case'1':

		CAR_FirstSpeed();
	break;
	case'2':

		CAR_SecondSpeed();
	break;
	case'3':

		CAR_ThirdSpeed();
	break;
	case'4':

		CAR_FourthSpeed();
	break;
	case'5':

		CAR_FifthSpeed();
	break;
	default:
		break;
	}
}
void Voltage_Read(void)
{	u32 voltage = 0;
	voltage = Volt_SensorReading(ADC_Reading);
	USART6_SendNumberINT(ADC_Reading);
}
