/*
 * ADC_Prv.h
 *
 *  Created on: Sep 15, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_ADC_PRV_H_
#define INC_ADC_PRV_H_

#define ADC1_BASE_ADDRESS		0x40012000

typedef struct
{
	u32 SR;
	u32 CR1;
	u32 CR2;
	u32 SMPR1;
	u32 SMPR2;
	u32 JOFR1;
	u32 JOFR2;
	u32 JOFR3;
	u32 JOFR4;
	u32 HTR;
	u32 LTR;
	u32 SQR1;
	u32 SQR2;
	u32 SQR3;
	u32 JSQR;
	u32 JDR1;
	u32 JDR2;
	u32 JDR3;
	u32 JDR4;
	u32 DR;
	u32 CCR;

}ADC_t;

#define ADC1 ((volatile ADC_t *)( ADC1_BASE_ADDRESS))
#define RES_MUSK		0b11
#define PRE_MUSK		0b11
#define SQR_MUSK		0b11111
#define SMPR_MUSK		0b111
#define JSQR_MUSK		0b11111
#define EXTEN_MUSK		0b11
#define EXTSEL_MUSK		0b1111
#define JEXTEN_MUSK		0b11
#define JEXTSEL_MUSK	0b1111
#define DISCNUM_MUSK	0b111

#define SR_EOC			1
#define SR_JEOC			2

#define CR2_ADON		0
#define CR2_CONT		1
#define CR2_DMA			8
#define CR2_DDS			9
#define CR2_EOCS		10
#define CR2_JEXTSEL1	16
#define CR2_JEXTEN1		20
#define CR2_JSWSTART	22
#define CR2_EXTSEL1		24
#define CR2_EXTEN1		28
#define CR2_SWSTART		30

#define CR1_EOCIE		5
#define CR1_JEOCIE		7
#define CR1_SCAN		8
#define CR1_JAUTO		10
#define CR1_DISCEN		11
#define CR1_JDISCEN		12
#define CR1_DISCNUM1	13
#define SQR1_L			20
#endif /* INC_ADC_PRV_H_ */
