/*
 * NVIC_Prv.h
 *
 *  Created on: Jul 27, 2024
 *      Author: Mostafa Hisham
 */

#ifndef NVIC_PRV_H_
#define NVIC_PRV_H_

#define NVIC_BASE_ADDRESS	0xE000E100

typedef struct{
	u32 ISER[8];
	u32 Reserved_0[24];
	u32 ICER[8];
	u32 Reserved_1[24];
	u32 ISPR[8];
	u32 Reserved_2[24];
	u32 ICPR[8];
	u32 Reserved_3[24];
	u32 IABR[8];
	u32 Reserved_4[56];
	u8 IPR[240];
	u32 Reserved_5[580];
	u32 STIR;
}NVIC_t;

#define NVIC  			((volatile NVIC_t*)(NVIC_BASE_ADDRESS))
#define AIRCR 			*((volatile u32*)(0xE000ED0C))
#define NVIC_DIVISION 	32
#define NVIC_REMINDER 	32
#define VECTKEY 		0X05FA0000


#endif /* NVIC_PRV_H_ */
