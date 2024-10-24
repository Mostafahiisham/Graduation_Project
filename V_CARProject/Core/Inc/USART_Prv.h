/*
 * USART_Prv.h
 *
 *  Created on: Aug 24, 2024
 *      Author: Mostafa Hisham
 */

#ifndef INC_USART_PRV_H_
#define INC_USART_PRV_H_

#define USART1_BASE_ADDRESS			0x40011000
#define USART6_BASE_ADDRESS			0x40011400

typedef struct
{
	u32 SBK :1;
	u32 RWU:1;
	u32 RE:1;
	u32 TE:1;
	u32 IDLEIE:1;
	u32 RXNEIE:1;
	u32 TCIE:1;
	u32 TXEIE:1;
	u32 PEIE:1;
	u32 PS:1;
	u32 PCE:1;
	u32 WAKE:1;
	u32 M :1;
	u32 UE:1;
	u32 RESERVED:1;
	u32 OVER8:1;
	u32 RESERVED_2:16;

}CR1_t;

typedef struct{
	u32 SR;
	u32 DR;
	u32 BRR;
	CR1_t CR1;
	u32 CR2;
	u32 CR3;
	u32 GTPR;
}USART_t;

#define USART1 ((volatile USART_t*)(USART1_BASE_ADDRESS))
#define USART6 ((volatile USART_t*)(USART6_BASE_ADDRESS))

#define SR_RXNE				5
#define SR_TC				6
#define SR_TXE				7


#endif /* INC_USART_PRV_H_ */
