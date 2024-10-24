/*
 * MGPIO_Prv.h
 *
 *  Created on: Jul 20, 2024
 *      Author: Mostafa Hisham
 */

#ifndef MGPIO_PRV_H_
#define MGPIO_PRV_H_

#define GPIOA_BASE_ADDRESS		0x40020000
#define GPIOB_BASE_ADDRESS		0x40020400
#define GPIOC_BASE_ADDRESS		0x40020800

typedef struct{

	u32 MODER;
	u32 OTYPER;
	u32 OSPEEDR;
	u32 PUPDR;
	u32 IDR;
	u32 ODR;
	u32 BSRR;
	u32 LCKR;
	u32 AFR[2];
}GPIOX_t;
#define GPIOA   ((volatile GPIOX_t*) (GPIOA_BASE_ADDRESS))
#define GPIOB   ((volatile GPIOX_t*) (GPIOB_BASE_ADDRESS))
#define GPIOC   ((volatile GPIOX_t*) (GPIOC_BASE_ADDRESS))

#define MODE_MUSK 	0b11
#define MODE_BIT_ACCESS 2u

#define SPEED_MUSK 	0b11
#define SPEED_BIT_ACCESS 2u

#define PUPDR_MUSK 0b11
#define PUPDR_BIT_ACCESS 2u
#define AFR_MUSK 0b1111
#define AFR_BIT_ACCESS  4u
#endif /* MGPIO_PRV_H_ */
