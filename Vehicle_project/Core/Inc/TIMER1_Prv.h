/*
 * TIMER1_Prv.h
 *
 *  Created on: Aug 13, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_TIMER1_PRV_H_
#define INC_TIMER1_PRV_H_

#define TIMER1_BASE_ADDRESS 0x40010000

typedef struct{
	u32 CR1;
	u32 CR2;
	u32 SMCR;
	u32 DIER;
	u32 SR;
	u32 EGR;
	u32 CCMR1;
	u32 CCMR2;
	u32 CCER;
	u32 CNT;
	u32 PSC;
	u32 ARR;
	u32 RCR;
	u32 CCR1;
	u32 CCR2;
	u32 CCR3;
	u32 CCR4;
	u32 BDTR;
	u32 DCR;
	u32 DMAR;
}TIMER1_t;


#define TIMER1 ((volatile TIMER1_t*)(TIMER1_BASE_ADDRESS))

#define OCM_MUSK		0b111

#define CR1_CEN			0
#define	CR1_URS			2
#define CR1_OPM 		3
#define CR1_ARPE		7

#define EGR_UG			0

#define DIER_UIE		0

#define SR_UIF			0

#define CCMR1_OC1PE		3
#define	CCMR1_OC2PE		11
#define	CCMR2_OC3PE		3
#define	CCMR2_OC4PE		11

#define CCER_CC1E		0
#define CCER_CC1P		1
#define CCER_CC1NP		3
#define	CCER_CC2E		4
#define CCER_CC2P		5
#define CCER_CC2NP		7
#define CCER_CC3E		8
#define CCER_CC3P		9
#define CCER_CC3NP		11
#define CCER_CC4E		12
#define CCER_CC4P		13

#define BDTR_MOE		15
#endif /* INC_TIMER1_PRV_H_ */
