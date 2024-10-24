/*
 * MRCC_Prv.h
 *
 *  Created on: Jul 19, 2024
 *      Author: Mostafa Hisham
 *      Layer : MCAL
 */

#ifndef MRCC_PRV_H_
#define MRCC_PRV_H_

#define RCC_BASE_ADDRESS 		0x40023800

#define RCC				((volatile MRCC_t*)(RCC_BASE_ADDRESS))

typedef struct{
	u32 RCC_CR;				//1
	u32 RCC_PLLCFGR;		//2
	u32 RCC_CFGR;			//3
	u32 RCC_CIR;			//4
	u32 RCC_AHB1RSTR;		//5
	u32 RCC_AHB2RSTR;		//6
	u32 Reserved_1[2];
	u32 RCC_APB1RSTR;		//7
	u32 RCC_APB2RSTR;		//8
	u32 Reserved_2[2];
	u32 RCC_AHB1ENR;		//9
	u32 RCC_AHB2ENR;		//10
	u32 Reserved_3[2];
	u32 RCC_APB1ENR;		//11
	u32 RCC_APB2ENR;		//2
	u32 Reserved_4[2];
	u32 RCC_AHB1LPENR;		//13
	u32 RCC_AHB2LPENR;		//14
	u32 Reserved_5[2];
	u32 RCC_APB1LPENR;		//15
	u32 RCC_APB2LPENR;		//16
	u32 Reserved_6[2];
	u32 RCC_BDCR;			//17
	u32 RCC_CSR;			//18
	u32 Reserved_7[2];
	u32 RCC_SSCGR;			//19
	u32 RCC_PLLI2SCFGR;		//20
	u32 Reserved_8;
	u32 RCC_DCKCFGR;		//21

}MRCC_t;

#define HSI_ON		0			/*high Speed Internal Clock On*/
#define HSI_RDY 	1			/*high Speed Internal  Ready*/
#define HSE_ON		16			/*high Speed External Clock On*/
#define HSE_RDY		17			/*high Speed External  Ready*/
#define HSE_BYP		18			/*high Speed Internal  ByBass Pin*/
#define RCC_CFGR_MASK 			0xFFFFFFFC
/*
#define RCC_AHB1ENR_MASK		0xFF9FEF60	//1111 1111 1001 1111 1110 1111 0110 0000
#define RCC_APB1ENR_MASK		0xEF1D37F0	//1110 1111 0001 1101 0011 0111 1111 0000
#define RCC_APB2ENR_MASK		0xFFF886CE	//1111 1111 1111 1000 1000 0110 1100 1110
#define RCC_AHB1LPENR_MASK		0xFF9E6F60	//1111 1111 1001 1110 0110 1111 0110 0000
#define RCC_APB1LPENR_MASK 		0xEF1D37F0	//1110 1111 0001 1101 0011 0111 1111 0000
#define RCC_APB2LPENR_MASK		0xFFF886CE	//1111 1111 1111 1000 1000 0110 1100 1110
*/


#endif /* MRCC_PRV_H_ */
