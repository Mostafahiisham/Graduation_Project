/*
 * MRCC_Int.h
 *
 *  Created on: Jul 19, 2024
 *      Author: Mostafa Hisham
 *      Layer : MCAL
 */

#ifndef MRCC_H_
#define MRCC_H_


						/***Type Of Clk***/
#define RC_Clk				1
#define External_Clk		2
						/**Choose Internal or External Clk**/
#define HSI_Clk				1
#define HSE_Clk				2
						/***Switch BEtween Types of Clk***/
#define HSI_SW_CLK			0
#define HSE_SW_CLK			1
#define PLL_SW_CLK			2

						/***Peripherals in AHB1 ***/

#define AHB1_GPIOA_EN		0
#define AHB1_GPIOB_EN		1
#define AHB1_GPIOC_EN		2
#define AHB1_GPIOD_EN		3
#define AHB1_GPIOE_EN		4
#define AHB1_GPIOH_EN		7
#define AHB1_CRC_EN			12
#define AHB1_DMA1_EN		21
#define AHB1_DMA2_EN		22
						/***Peripherals in AHB2 ***/
#define AHB2_OTGFS_EN		7
						/***Peripherals in APB1 ***/
#define APB1_TIM2_EN		0
#define APB1_TIM3_EN		1
#define APB1_TIM4_EN		2
#define APB1_TIM5_EN		3
#define APB1_WWDG_EN		11
#define APB1_SPI2_EN		14
#define APB1_SPI3_EN		15
#define APB1_USART2_EN		17
#define APB1_I2C1_EN		21
#define APB1_I2C2_EN		22
#define APB1_I2C3_EN		23
#define APB1_PWR_EN			28
						/***Peripherals in APB2 ***/
#define APB2_TIM1_EN		0
#define APB2_USART1_EN		4
#define APB2_USART6_EN		5
#define APB2_ADC1_EN		8
#define APB2_SDIO_EN		11
#define APB2_SPI1_EN		12
#define APB2_SPI4_EN		13
#define APB2_SYSCFG_EN		14
#define APB2_TIM9_EN		16
#define APB2_TIM10_EN		17
#define APB2_TIM11_EN		18


						/***Peripherals in AHB1-LP ***/

#define AHB1LP_GPIOALP_EN		0
#define AHB1LP_GPIOBLP_EN		1
#define AHB1LP_GPIOCLP_EN		2
#define AHB1LP_GPIODLP_EN		3
#define AHB1LP_GPIOELP_EN		4
#define AHB1LP_GPIOHLP_EN		7
#define AHB1LP_CRCLP_EN			12
#define AHB1LP_FLITF_LPEN		15
#define AHB1LP_SRAM1_LPEN		16
#define AHB1LP_DMA1_EN			21
#define AHB1LP_DMA2_EN			22
						/***Peripherals in AHB2-LP ***/
#define AHB2LP_OTGFSLP_EN		7
						/***Peripherals in APB1-LP ***/
#define APB1LP_TIM2LP_EN		0
#define APB1LP_TIM3LP_EN		1
#define APB1LP_TIM4LP_EN		2
#define APB1LP_TIM5LP_EN		3
#define APB1LP_WWDGLP_EN		11
#define APB1LP_SPI2LP_EN		14
#define APB1LP_SPI3LP_EN		15
#define APB1LP_USART2LP_EN		17
#define APB1LP_I2C1LP_EN		21
#define APB1LP_I2C2LP_EN		22
#define APB1LP_I2C3LP_EN		23
#define APB1LP_PWRLP_EN			28
						/***Peripherals in APB2-LP ***/
#define APB2LP_TIM1LP_EN		0
#define APB2LP_USART1LP_EN		4
#define APB2LP_USART6LP_EN		5
#define APB2LP_ADC1LP_EN		8
#define APB2LP_SDIOLP_EN		11
#define APB2LP_SPI1LP_EN		12
#define APB2LP_SPI4LP_EN		13
#define APB2LP_SYSCFGLP_EN		14
#define APB2LP_TIM9LP_EN		16
#define APB2LP_TIM10LP_EN		17
#define APB2LP_TIM11LP_EN		18



						/***Buses Types ***/
typedef enum{
	AHB1ENR,
	AHB2ENR,
	APB1ENR,
	APB2ENR,
	AHB1LPENR,
	AHB2LPENR,
	APB1LPENR,
	APB2LPENR
}Base_Type;

void RCC_VInit(void);
void RCC_VEnableClkPeripheral(u8 Copy_u8AddressBus,u8 Copy_u8Peripheral);
void RCC_VDisableClkPeripheral(Base_Type Copy_AddressBus,u8 Copy_u8Peripheral);






#endif /* MRCC_H_ */
