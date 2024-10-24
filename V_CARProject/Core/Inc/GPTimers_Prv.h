/*
 * GPTimers_Prv.h
 *
 *  Created on: Aug 15, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_GPTIMERS_PRV_H_
#define INC_GPTIMERS_PRV_H_

#define TIMER2_BASE_ADDRESS		0x40000000
#define TIMER3_BASE_ADDRESS		0x40000400
#define TIMER4_BASE_ADDRESS		0x40000800
#define TIMER5_BASE_ADDRESS		0x40000C00
#define TIMER9_BASE_ADDRESS		0x40014000
#define TIMER10_BASE_ADDRESS    0x40014400
#define TIMER11_BASE_ADDRESS    0x40014800
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
	u32 Reserved0;
	u32 CCR1;
	u32 CCR2;
	u32 CCR3;
	u32 CCR4;
	u32 Reserved1;
	u32 DCR;
	u32 DMAR;
	u32 OR;
}GP_TIMERx_t;

#define TIMER2	((volatile GP_TIMERx_t*)(TIMER2_BASE_ADDRESS))
#define TIMER3	((volatile GP_TIMERx_t*)(TIMER3_BASE_ADDRESS))
#define TIMER4	((volatile GP_TIMERx_t*)(TIMER4_BASE_ADDRESS))
#define TIMER5	((volatile GP_TIMERx_t*)(TIMER5_BASE_ADDRESS))
#define TIMER9  ((volatile GP_TIMERx_t*)(TIMER9_BASE_ADDRESS))
#define TIMER10 ((volatile GP_TIMERx_t*)(TIMER10_BASE_ADDRESS))
#define TIMER11 ((volatile GP_TIMERx_t*)(TIMER11_BASE_ADDRESS))

#define MMS_MUSK		0b111

#define CR1_CEN			0
#define	CR1_URS			2
#define CR1_OPM 		3
#define CR1_ARPE		7

#define CR2_MMS1		4

#define EGR_UG			0

#define DIER_UIE		0

#define SR_UIF			0

#define CCMR1_OC1PE		3
#define	CCMR1_OC2PE		11
#define	CCMR2_OC3PE		3
#define	CCMR2_OC4PE		11
#endif /* INC_GPTIMERS_PRV_H_ */
