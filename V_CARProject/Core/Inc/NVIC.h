/*
 * NVIC.h
 *
 *  Created on: Jul 27, 2024
 *      Author: Mostafa Hisham
 */

#ifndef NVIC_H_
#define NVIC_H_


typedef enum{
	GROUP16_SUB0 = 3,
	GROUP8_SUB2,
	GROUP4_SUB4,
	GROUP2_SUB8,
	GROUP0_SUB16
}NVIC_GROUP_Type;

typedef enum{
	WWDG,
	EXTI16_PVD,
	EXTI21_TAMP_STAMP,
	EXTI22_RTC_WKUP,
	FLASH,
	RCC,
	EXTI0,
	EXTI1,
	EXTI2,
	EXTI3,
	EXTI4,
	DMA1_Stream0,
	DMA1_Stream1,
	DMA1_Stream2,
	DMA1_Stream3,
	DMA1_Stream4,
	DMA1_Stream5,
	DMA1_Stream6,
	ADC,
	EXTI9_5 = 23,
	TIM1_BRK_TIM9,
	TIM1_UP_TIM10,
	TIM1_TRG_COM_TIM11,
	TIM1_CC,
	TIM2,
	TIM3,
	TIM4,
	I2C1_EV,
	I2C1_ER,
	I2C2_EV,
	I2C2_ER,
	SPI1,
	SPI2,
	USART1,
	USART2,
	EXTI15_10 = 40,
	EXTI17_RTC_Alarm,
	EXTI18_OTG_FS_WKUP,
	DMA1_Stream7 = 47,
	SDIO = 49,
	TIM5,
	SPI3,
	DMA2_Stream0 = 56,
	DMA2_Stream1,
	DMA2_Stream2,
	DMA2_Stream3,
	DMA2_Stream4,
	OTG_FS = 67,
	DMA2_Stream5,
	DMA2_Stream6,
	DMA2_Stream7,
	USART6,
	I2C3_EV,
	I2C3_ER,
	FPU = 81,
	SPI4 = 84
}INTERRUPT_Position;

u8 NVIC_EnableInterrupt(INTERRUPT_Position Copy_uddtPosition);

u8 NVIC_DisableInterrupt(INTERRUPT_Position Copy_uddtPosition);

u8 NVIC_SetInterruptPriority(u8 Copy_u8GroupNum,u8 Copy_u8SubNum,INTERRUPT_Position Copy_uddtPosition);

u8 NVIC_SelectGroupNumber(NVIC_GROUP_Type Copy_uddtGroupType);




#endif /* NVIC_H_ */
